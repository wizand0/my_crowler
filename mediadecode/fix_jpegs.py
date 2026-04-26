# fix_jpegs.py
# Использование: python fix_jpegs.py /путь/к/REC_0

import sys
import os
import shutil

def fix_jpeg(path):
    with open(path, 'rb') as f:
        data = f.read()

    # Нет SOI → безнадёжно битый
    if len(data) < 4 or data[0:2] != b'\xff\xd8':
        return 'broken'

    # Ищем EOI (последнее вхождение FF D9)
    eoi = data.rfind(b'\xff\xd9')
    if eoi == -1:
        # EOI нет — дописываем, получим обрезанную но просматриваемую картинку
        with open(path, 'ab') as f:
            f.write(b'\xff\xd9')
        return 'truncated_fixed'

    # Есть мусор после EOI — обрезаем
    if eoi + 2 < len(data):
        with open(path, 'wb') as f:
            f.write(data[:eoi + 2])
        return 'trimmed'

    return 'ok'

def main(folder):
    broken_dir = os.path.join(folder, 'broken')
    os.makedirs(broken_dir, exist_ok=True)

    stats = {'ok': 0, 'truncated_fixed': 0, 'trimmed': 0, 'broken': 0}

    for name in sorted(os.listdir(folder)):
        if not name.lower().endswith('.jpg'):
            continue
        path = os.path.join(folder, name)
        if not os.path.isfile(path):
            continue

        result = fix_jpeg(path)
        stats[result] += 1

        if result == 'broken':
            shutil.move(path, os.path.join(broken_dir, name))
        print(f'{name}: {result}')

    print('\n=== Итого ===')
    for k, v in stats.items():
        print(f'  {k}: {v}')

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Использование: python fix_jpegs.py /путь/к/папке')
        sys.exit(1)
    main(sys.argv[1])