from time import sleep

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
ut = conn.add_stream(getattr, conn.space_center, 'ut')  # Универсальное время по Курбину
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')  # Высота над уровнем моря
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')  # Апогей
periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')  # Перигей
start_time = ut()


def get_time():
    curr_time = math.ceil(ut() - start_time)
    hours = curr_time // (60 ** 2)
    minutes = curr_time % (60 ** 2) // 60
    seconds = curr_time % 60

    def form(s):
        return f"0{s}" if len(str(s)) == 1 else s

    return f"{form(hours)}:{form(minutes)}:{form(seconds)}"


def log(message):
    print(f"[{get_time()}] {message}")


# Параметры Кёрбина
MU_KERBIN = 3.5316e12  # гравитационный параметр (м³/с²)
RADIUS_KERBIN = 600000  # радиус Кёрбина (м)
ROTATION_PERIOD_KERBIN = 21600  # период вращения Кёрбина (с)

# Высота геостационарной орбиты
GEOSTATIONARY_ALTITUDE = (MU_KERBIN * (ROTATION_PERIOD_KERBIN ** 2) / (4 * math.pi ** 2)) ** (1 / 3) - RADIUS_KERBIN
print(f"Геостационарная орбита Кёрбина: {GEOSTATIONARY_ALTITUDE:.2f}м.")

# Запуск
log("Запуск двигателей...")
ap.engage()
ap.target_pitch_and_heading(90, 90)  # Вертикальный старт.
control.activate_next_stage()
for i in range(3, 0, -1):
    log(f"{i}...")
    sleep(1)
control.throttle = 1.0

# Выход на высоту геостационарной орбиты
while apoapsis() < GEOSTATIONARY_ALTITUDE:
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
        ap.engage()  # Повторное включение автопилота

    # Постепенный наклон для оптимизации траты топлива и выхода на орбиту
    if altitude() > 10000:
        control.throttle = 1.0
        ap.target_pitch_and_heading(45, 90)
    if altitude() > 20000:
        ap.target_pitch_and_heading(30, 90)
    if altitude() > 30000:
        ap.target_pitch_and_heading(20, 90)

    time.sleep(0.1)

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
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time / 2)
while ut() < burn_ut:
    time.sleep(0.1)

# Коррекция орбиты до идеальной круговой
log("Начало маневра.")
ap.target_pitch_and_heading(0, 90)
control.throttle = 1.0
while periapsis() + apoapsis() < 2 * GEOSTATIONARY_ALTITUDE:
    sleep(0.1)
control.throttle = 0
node.remove()  # Выключение разметки идеальной орбиты
log("Манёвр завершен.")
log("Выход на геостационарную орбиту выполнен!")
log("Поворот спутника в сторону Кёрбина...")
ap.target_pitch_and_heading(-90, 90)
sleep(20)
log("Миссия по выводу спутника на геостационарную орбиту завершена.")
