"""
Генерація самопідписаного TLS-сертифіката для хаба.

Чому самопідписаний: Let's Encrypt видає сертифікати лише на справжні доменні
імена з публічним DNS. Для 192.168.1.x чи rpi5.local його не отримати
в принципі. Тому шифрування ми маємо повноцінне, але браузер не знає, кому
довіряти, і показує попередження — його треба прийняти один раз на кожному
пристрої.

Коли з'явиться справжній домен, код сервера міняти НЕ доведеться: достатньо
покласти замість цих файлів сертифікат від Let's Encrypt.

Ключовий момент — SAN (Subject Alternative Name). Сучасні браузери повністю
ігнорують поле CN і дивляться ТІЛЬКИ в SAN. Якщо адреси, за якою ти відкриваєш
сторінку, немає в списку SAN — з'єднання буде відхилене навіть після того,
як ти прийняв сертифікат. Тому сюди треба вписати всі імена й IP, за якими
хаб може бути доступний: і localhost, і LAN-адресу ноутбука, і майбутню
адресу RPi5.

  python hub/make_cert.py
  python hub/make_cert.py --extra rpi5.local --extra 192.168.1.77
  python hub/make_cert.py --force        # перевипустити, затерши старий
"""

from __future__ import annotations

import argparse
import datetime as dt
import ipaddress
import socket
import sys
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SECRETS_DIR = PROJECT_ROOT / "secrets"
CERT_PATH = SECRETS_DIR / "cert.pem"
KEY_PATH = SECRETS_DIR / "key.pem"

# 397 днів — межа, після якої браузери (Safari, Chrome) відхиляють сертифікат
# незалежно від того, що в ньому написано. Робити довший немає сенсу.
VALID_DAYS = 397


def local_ipv4_addresses() -> list[str]:
    """Усі не-loopback IPv4 цієї машини."""
    addrs = set()
    hostname = socket.gethostname()
    try:
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                addrs.add(ip)
    except socket.gaierror:
        pass
    return sorted(addrs)


def build_san(extra: list[str]) -> tuple[x509.SubjectAlternativeName, list[str]]:
    """Складає список SAN, розділяючи імена та IP-адреси."""
    names: list[x509.GeneralName] = []
    described: list[str] = []

    hostname = socket.gethostname()
    dns_names = {"localhost", hostname, f"{hostname}.local"}
    ip_addrs = {"127.0.0.1", "::1", *local_ipv4_addresses()}

    for item in extra:
        try:
            ipaddress.ip_address(item)
            ip_addrs.add(item)
        except ValueError:
            dns_names.add(item)

    for name in sorted(dns_names):
        names.append(x509.DNSName(name))
        described.append(f"DNS:{name}")

    for ip in sorted(ip_addrs):
        try:
            names.append(x509.IPAddress(ipaddress.ip_address(ip)))
            described.append(f"IP:{ip}")
        except ValueError:
            print(f"[cert] пропускаю некоректну адресу: {ip}")

    return x509.SubjectAlternativeName(names), described


def main() -> int:
    ap = argparse.ArgumentParser(description="Самопідписаний TLS-сертифікат для хаба")
    ap.add_argument("--extra", action="append", default=[],
                    help="додаткове ім'я або IP у SAN (можна кілька разів). "
                         "Сюди вписуй майбутню адресу RPi5")
    ap.add_argument("--force", action="store_true", help="перевипустити, затерши старий")
    args = ap.parse_args()

    SECRETS_DIR.mkdir(parents=True, exist_ok=True)

    if CERT_PATH.exists() and not args.force:
        print(f"[cert] {CERT_PATH} уже існує. Щоб перевипустити — додай --force")
        return 0

    # EC P-256 замість RSA: коротший ключ при тій самій стійкості й помітно
    # менше навантаження на CPU при рукостисканні. Для RPi5 це не дрібниця.
    key = ec.generate_private_key(ec.SECP256R1())

    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COMMON_NAME, "VideoDetection hub"),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "VideoDetection"),
    ])

    san, described = build_san(args.extra)
    now = dt.datetime.now(dt.timezone.utc)

    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - dt.timedelta(minutes=5))  # запас на розбіжність годинників
        .not_valid_after(now + dt.timedelta(days=VALID_DAYS))
        .add_extension(san, critical=False)
        .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
        .add_extension(
            x509.KeyUsage(
                digital_signature=True, key_encipherment=False, key_agreement=True,
                content_commitment=False, data_encipherment=False, key_cert_sign=False,
                crl_sign=False, encipher_only=False, decipher_only=False,
            ),
            critical=True,
        )
        .add_extension(
            x509.ExtendedKeyUsage([x509.ObjectIdentifier("1.3.6.1.5.5.7.3.1")]),  # serverAuth
            critical=False,
        )
        .sign(key, hashes.SHA256())
    )

    CERT_PATH.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    KEY_PATH.write_bytes(
        key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.PKCS8,
            encryption_algorithm=serialization.NoEncryption(),
        )
    )

    print(f"[cert] сертифікат : {CERT_PATH}")
    print(f"[cert] ключ       : {KEY_PATH}")
    print(f"[cert] дійсний до : {(now + dt.timedelta(days=VALID_DAYS)):%Y-%m-%d}")
    print(f"[cert] SAN        : {', '.join(described)}")
    print("\n[cert] Відкривати хаб можна ТІЛЬКИ за адресами зі списку SAN.")
    print("[cert] Для іншої адреси перевипусти сертифікат з --extra <адреса> --force")
    return 0


if __name__ == "__main__":
    sys.exit(main())
