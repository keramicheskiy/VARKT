from time import sleep

import sys
import krpc
import time
import math

"""
Автор - Шарков И.П.(М8О-115БВ-24, Московский Авиационный Институт).
Специально для проекта по предмету «Введение в авиационную, ракетную и космическую технику».
"""

# Установка соединения с KSP и получение объектов для управления космическим кораблем
conn = krpc.connect()
vessel = conn.space_center.active_vessel
ap = vessel.auto_pilot
control = vessel.control

# Получение потоков данных для мониторинга необходимых параметров
ut = conn.add_stream(getattr, conn.space_center, 'ut')  # Универсальное время по Кёрбину
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')  # Высота над уровнем моря
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')  # Апогей
periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')  # Перигей
horizontal_speed = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'horizontal_speed')
vertical_speed = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'vertical_speed')
speed = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'speed')
start_time = ut()

log_destination = open("logs.txt", "a", encoding="UTF-8")  # Файл для записи логов
log_span = 1  # Интервал времени между выводами логов

def get_time():
    curr_time = math.ceil(ut() - start_time)
    hours = curr_time // (60 ** 2)
    minutes = curr_time % (60 ** 2) // 60
    seconds = curr_time % 60

    def form(s):
        return f"0{s}" if len(str(s)) == 1 else s

    return f"{form(hours)}:{form(minutes)}:{form(seconds)}"


def log(message):
    sys.stdout = log_destination
    print(f"[{get_time()}] {message}")
    sys.stdout = sys.__stdout__
    print(f"[{get_time()}] {message}")


def log_measures(fuel_consumption):
    measures = {
        "Время (с)": math.ceil(ut() - start_time),
        "Масса (кг)": round(vessel.mass, 2),
        "Высота (м)": round(altitude(), 2),
        "Расход топлива (кг/с)": round(fuel_consumption, 2),
        "Скорость (м/с)": round(speed(), 2),
        "Вертикальная скорость (м/с)": round(vertical_speed(), 2),
        "Горизонтальная скорость (м/с)": round(horizontal_speed(), 2),
    }
    log(measures)


# Параметры Кёрбина
MU_KERBIN = 3.5316e12  # гравитационный параметр (м³/с²)
RADIUS_KERBIN = 600000  # радиус Кёрбина (м)
ROTATION_PERIOD_KERBIN = 21600  # период вращения Кёрбина (с)

# Высота геостационарной орбиты
GEOSTATIONARY_ALTITUDE = (MU_KERBIN * (ROTATION_PERIOD_KERBIN ** 2) / (4 * math.pi ** 2)) ** (1 / 3) - RADIUS_KERBIN
print(f"Геостационарная орбита Кёрбина: {GEOSTATIONARY_ALTITUDE:.2f}м.")

# Запуск
log("Запуск через")
ap.engage()
ap.target_pitch_and_heading(90, 90)  # Вертикальный старт.
control.activate_next_stage()
for i in range(3, 0, -1):
    log(f"{i}...")
    sleep(1)
control.throttle = 1.0

# Выход на высоту геостационарной орбиты
flag10km = 0
flag20km = 0
flag30km = 0
last_fuel_log_time = 0
while apoapsis() < GEOSTATIONARY_ALTITUDE:
    current_time = ut()
    if current_time - last_fuel_log_time > log_span:
        thrust = vessel.available_thrust
        isp = vessel.specific_impulse
        if thrust > 0 and isp > 0:
            flow_rate = thrust / (isp * 9.82)  # Формула расхода топлива
            log_measures(flow_rate)
        last_fuel_log_time = current_time

    curr_stage = vessel.control.current_stage
    resources = vessel.resources_in_decouple_stage(stage=curr_stage - 1, cumulative=False)
    liquid_fuel = resources.amount('LiquidFuel')
    oxidizer = resources.amount('Oxidizer')
    total_fuel = liquid_fuel + oxidizer

    if total_fuel <= 1 and curr_stage > 2:
        if curr_stage == 9:
            log("Поехали!")
        elif curr_stage in [5, 6]:
            log("Отсоединение носового обтекателя.")
        else:
            log(f"Отделение ступени {curr_stage}.")
        control.activate_next_stage()
        time.sleep(1)
        ap.engage()

    # Постепенный наклон для оптимизации траты топлива и выхода на орбиту
    if 10000 < altitude() < 20000 and not flag10km:
        log("Высота 10 км достигнута, наклон ракеты на 45 градусов относительно горизонта.")
        control.throttle = 1.0
        ap.target_pitch_and_heading(45, 90)
        flag10km = 1
    if 20000 < altitude() < 30000 and not flag20km:
        log("Высота 20 км достигнута, наклон ракеты на 30 градусов относительно горизонта.")
        ap.target_pitch_and_heading(30, 90)
        flag20km = 1
    if 30000 < altitude() and not flag30km:
        log("Высота 30 км достигнута, наклон ракеты на 20 градусов относительно горизонта.")
        ap.target_pitch_and_heading(20, 90)
        flag30km = 1

    time.sleep(0.01)

control.throttle = 0
log("Достигнута необходимая высота апогея, ожидание точки манёвра...")

# Создание узла маневра для круговой орбиты
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
v_circular = math.sqrt(mu / r)
v_current = math.sqrt(mu * (2 / r - 1 / vessel.orbit.semi_major_axis))
delta_v = v_circular - v_current
node = control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Расчет времени сжигания топлива
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v / Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

# Ожидание до начала маневра
last_fuel_log_time = 0
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time / 2)
while ut() < burn_ut:
    current_time = ut()
    if current_time - last_fuel_log_time > log_span:
        thrust = vessel.available_thrust
        isp = vessel.specific_impulse
        if thrust > 0 and isp > 0:
            flow_rate = thrust / (isp * 9.82)  # Формула расхода топлива
            log_measures(flow_rate)
        last_fuel_log_time = current_time
    time.sleep(0.01)

# Коррекция орбиты до идеальной круговой
log("Начало маневра.")
ap.target_pitch_and_heading(0, 90)
control.throttle = 1.0
last_fuel_log_time = 0
while periapsis() + apoapsis() < 2 * GEOSTATIONARY_ALTITUDE:
    current_time = ut()
    if current_time - last_fuel_log_time > log_span:
        thrust = vessel.available_thrust
        isp = vessel.specific_impulse
        if thrust > 0 and isp > 0:
            flow_rate = thrust / (isp * 9.82)  # Формула расхода топлива
            log_measures(flow_rate)
        last_fuel_log_time = current_time
    sleep(0.01)
control.throttle = 0
node.remove()  # Выключение разметки идеальной орбиты
log("Манёвр завершен.")
log("Выход на геостационарную орбиту выполнен!")
log("Поворот спутника в сторону Кёрбина...")
ap.target_pitch_and_heading(-90, 90)
sleep(20)
log("Миссия по выводу спутника на геостационарную орбиту завершена.")
