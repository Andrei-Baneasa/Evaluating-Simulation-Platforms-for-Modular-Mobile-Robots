import re

INPUT_FILE = r"C:\Users\Andrei_Baneasa\Downloads\Evaluating-Simulation-Platforms-for-Modular-Mobile-Robots\benchmark\WeBots\benchmark\webots_perf.txt"
OUTPUT_FILE = r"C:\Users\Andrei_Baneasa\Downloads\Evaluating-Simulation-Platforms-for-Modular-Mobile-Robots\benchmark\WeBots\benchmark\webots_extracted.csv"
BASIC_TIME_STEP_S = 0.032  # 32 ms

with open(INPUT_FILE, "r", encoding="utf-8") as f:
    text = f.read()

blocks = text.split("World:")[1:]

rows = []

for i, block in enumerate(blocks, start=1):
    speed_match = re.search(r"Average speed factor:\s*([\d.]+)x", block)
    tot_match = re.search(
        r"TOT\s+(\d+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)"
        r"\s+[\d.]+\s+[\d.]+\s+[\d.]+\s+[\d.]+\s+([\d.]+)",
        block
    )

    if not speed_match or not tot_match:
        continue

    speed_factor = float(speed_match.group(1))
    steps = int(tot_match.group(1))
    loading_ms = float(tot_match.group(2))
    pre_physics_ms = float(tot_match.group(3))
    physics_ms = float(tot_match.group(4))
    post_physics_ms = float(tot_match.group(5))
    rendering_ms = float(tot_match.group(6))
    controller_ms = float(tot_match.group(7))

    simulated_time_s = steps * BASIC_TIME_STEP_S

    wall_time_no_loading_s = (
        pre_physics_ms
        + physics_ms
        + post_physics_ms
        + rendering_ms
        + controller_ms
    ) / 1000.0

    wall_time_with_loading_s = wall_time_no_loading_s + loading_ms / 1000.0

    real_time_factor = simulated_time_s / wall_time_no_loading_s

    rows.append([
        i,
        steps,
        simulated_time_s,
        wall_time_no_loading_s,
        wall_time_with_loading_s,
        real_time_factor,
        speed_factor,
        loading_ms,
        pre_physics_ms,
        physics_ms,
        post_physics_ms,
        rendering_ms,
        controller_ms,
    ])

with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
    f.write(
        "Run,Steps,Simulated Time (s),Wall Time No Loading (s),"
        "Wall Time With Loading (s),Real-Time Factor,Webots Speed Factor,"
        "Loading (ms),PrePhysics (ms),Physics (ms),PostPhysics (ms),"
        "Rendering (ms),Controller (ms)\n"
    )

    for row in rows:
        f.write(",".join(str(x) for x in row) + "\n")

print(f"Extracted {len(rows)} benchmark runs.")
print(f"Saved to {OUTPUT_FILE}")