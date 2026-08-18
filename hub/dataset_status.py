"""
Етап 6 — що вже зібрано і чого бракує.

Без цього легко зняти шість схожих сесій і виявити це вже після розмітки.
Скрипт нічого не змінює, лише показує стан і нагадує про перекоси.

  python hub/dataset_status.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RAW_DIR = PROJECT_ROOT / "datasets" / "raw"

# Орієнтири, а не догма. Виведені з того, що донавчання під вузький домен
# зазвичай починає давати вимірюваний ефект від кількох сотень кадрів.
TARGET_TOTAL = 800
TARGET_EMPTY_SHARE = 0.20      # частка кадрів без людей
MIN_SESSIONS = 5               # менше — нічого буде відкласти у валідацію


def main() -> int:
    if not RAW_DIR.exists():
        print(f"[dataset] ще нічого не зібрано: {RAW_DIR} не існує")
        print("[dataset] почни: python hub/collect.py --session day_backlit")
        return 0

    sessions = sorted(p for p in RAW_DIR.iterdir() if p.is_dir())
    if not sessions:
        print("[dataset] тек сесій немає")
        return 0

    total = total_hard = total_person = 0
    rows = []

    for s in sessions:
        meta_path = s / "session.json"
        meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}
        frames = list((s / "frames").glob("*.jpg")) if (s / "frames").exists() else []

        n = len(frames)
        hard = meta.get("frames_hard", 0)
        person = meta.get("frames_with_person_predicted", 0)

        total += n
        total_hard += hard
        total_person += person
        rows.append((s.name, n, hard, person, meta.get("note", ""),
                     meta_path.exists()))

    print(f"\n{'сесія':<20s} {'кадрів':>7s} {'складних':>9s} {'з людиною':>10s}  умови")
    print("-" * 78)
    for name, n, hard, person, note, has_meta in rows:
        share = f"{person / n:.0%}" if n else "—"
        flag = "" if has_meta else "  [!] немає session.json"
        print(f"{name:<20s} {n:7d} {hard:9d} {share:>10s}  {note[:28]}{flag}")
    print("-" * 78)
    print(f"{'РАЗОМ':<20s} {total:7d} {total_hard:9d} "
          f"{(total_person / total if total else 0):>9.0%}")

    empty = total - total_person
    empty_share = empty / total if total else 0

    print("\nстан:")
    print(f"  сесій {len(sessions)} (потрібно щонайменше {MIN_SESSIONS}: "
          "одну цілком відкладаємо у валідацію)")
    print(f"  кадрів {total} з орієнтовних {TARGET_TOTAL}")
    print(f"  без людей ~{empty} ({empty_share:.0%}), орієнтир {TARGET_EMPTY_SHARE:.0%}")

    print("\nщо варто врахувати:")
    if len(sessions) < MIN_SESSIONS:
        print(f"  - додай ще {MIN_SESSIONS - len(sessions)} сесій в ІНШИХ умовах "
              "(інше світло, інший час доби)")
    if total < TARGET_TOTAL:
        print(f"  - бракує ~{TARGET_TOTAL - total} кадрів")
    if empty_share < TARGET_EMPTY_SHARE * 0.7:
        print("  - замало порожніх кадрів: вони вчать модель НЕ спрацьовувати "
              "на халаті й тінях")
    if empty_share > TARGET_EMPTY_SHARE * 1.8:
        print("  - забагато порожніх кадрів: додай сесій з людиною")
    if total and total_hard / total > 0.4:
        print("  - складні кадри переважають: датасет перекошений у бік важких "
              "сцен, модель почне помилятися на простих")

    print("\nнагадування: частка «з людиною» — це ПЕРЕДБАЧЕННЯ поточної моделі,")
    print("а не істина. Саме там, де вона помилялась, і буде користь донавчання.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
