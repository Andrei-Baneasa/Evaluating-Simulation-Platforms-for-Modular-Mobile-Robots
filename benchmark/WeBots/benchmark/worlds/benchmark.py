import psutil
import subprocess
import time

WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD_FILE = r"C:\Users\Andrei_Baneasa\Downloads\Evaluating-Simulation-Platforms-for-Modular-Mobile-Robots\benchmark\WeBots\benchmark\worlds\connector_docking_benchmark.wbt"
print("Simulator,Wall Time (s),CPU Time (s),Average CPU (%),Average RAM (MB),Peak RAM (MB)")

LOG_FILE = r"C:\Users\Andrei_Baneasa\Downloads\webots_perf.txt"

webots = subprocess.Popen([
    WEBOTS_EXE,
    "--mode=fast",
    "--no-rendering",
    "--batch",
    "--stdout",
    "--stderr",
    f"--log-performance={LOG_FILE}",
    WORLD_FILE
])

proc = psutil.Process(webots.pid)

cpu_start = proc.cpu_times()
wall_start = time.perf_counter()

ram_samples = []

while webots.poll() is None:
    try:
        ram_samples.append(
            proc.memory_info().rss / (1024 * 1024)
        )
    except psutil.NoSuchProcess:
        break

    time.sleep(0.01)

wall_end = time.perf_counter()

try:
    cpu_end = proc.cpu_times()
except psutil.NoSuchProcess:
    cpu_end = cpu_start

wall_time = wall_end - wall_start

cpu_time = (
    (cpu_end.user + cpu_end.system)
    - (cpu_start.user + cpu_start.system)
)

avg_cpu = 100.0 * cpu_time / wall_time if wall_time > 0 else 0.0

avg_ram = (
    sum(ram_samples) / len(ram_samples)
    if ram_samples else 0.0
)

peak_ram = (
    max(ram_samples)
    if ram_samples else 0.0
)

print(
    f"Webots,"
    f"{wall_time:.6f},"
    f"{cpu_time:.6f},"
    f"{avg_cpu:.2f},"
    f"{avg_ram:.2f},"
    f"{peak_ram:.2f}"
)

