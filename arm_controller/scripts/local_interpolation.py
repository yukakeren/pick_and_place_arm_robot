import numpy as np

def interpolate(angle):
    # Pastikan sudut berada di dalam range
    if angle < -90.0: angle = -90.0
    elif angle > 90.0: angle = 90.0
    # Persamaan interpolasi linear dari range
    # [-90, 90] ke [1000, 5000]
    digital_unit = (angle + 90) * (5000 - 1000) / (90 - (-90)) + 1000
    # Kembalikan nilai sebagai integer
    return int(digital_unit)

def interpolate_np(angles):
    angle_range = (-90.0, 90.0)
    digital_range = (1000, 5000)
    digital_units = np.interp(angles, angle_range, digital_range)
    return digital_units.astype(int)

def main():
    # Sudut dalam derajat
    angles = [100.0, -90.0, 73.34]
    # Nilai digital dari sudut
    digital_units = [interpolate(angle) for angle in angles]
    # Tampilkan nilai sebelum dan setelah interpolasi
    print(f'Sudut: {angles}')
    print(f'Nilai : {digital_units}')
    digital_units_np = interpolate_np(angles)
    print(f'Nilai Numpy: {digital_units_np}')

if __name__ == '__main__':
    main()