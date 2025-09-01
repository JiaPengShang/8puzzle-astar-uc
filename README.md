# 8-Puzzle Solver (UC & A\*)

## Team

* **Jiapeng Shang** — *ID:* 23010093
* **Yuchen Dou** — *ID:* 23009960

## Overview

C++ implementations of **Uniform Cost Search** and **A\*** (with **Manhattan** and **Misplaced Tiles** heuristics) for the 8-Puzzle.
The solver uses:

* **Strict Expanded List** (no re-expansions)
* Fixed successor order **URDL** (Up, Right, Down, Left)
* A\* tie-breaker: **if `f` ties, prefer larger `g`**
* SDL2/SDL\_bgi animation for visualizing solutions

## Requirements

* C++14 compiler (e.g., **Apple clang ≥15** / **g++ ≥8.1**)
* **SDL2**
* **SDL\_bgi** (built for your CPU architecture; on Apple Silicon: `arm64`)

## Build

### macOS (Apple Silicon) — one-liner

```bash
clang++ -O2 -std=c++14 \
  -I/opt/homebrew/include -I/opt/homebrew/include/SDL2 \
  -I/usr/local/include    -I/usr/local/include/SDL2 \
  main.cpp algorithm.cpp puzzle.cpp \
  -L/opt/homebrew/lib -L/usr/local/lib \
  -lSDL_bgi -lSDL2 \
  -Wl,-rpath,/usr/local/lib -Wl,-rpath,/opt/homebrew/lib \
  -o search.out
```

> If you see “Library not loaded: libSDL\_bgi…”, run:
> `export DYLD_LIBRARY_PATH="/usr/local/lib:/opt/homebrew/lib:$DYLD_LIBRARY_PATH"`

## Run

### Single run (text mode)

```bash
./search.out "single_run" <algorithm> <start> <goal>
# Example:
./search.out "single_run" astar_explist_manhattan "608435127" "123456780"
```

### Animated run (visualization)

```bash
./search.out "animate_run" <algorithm> <start> <goal>
# Example:
./search.out "animate_run" astar_explist_manhattan "608435127" "123456780"
# Focus the SDL window and press any key to start the animation.
```

### Batch experiments (for grading)

```bash
./search.out "batch_run" all
# or save to a text file:
./search.out "batch_run" all > results_run.txt
```

**Algorithms (fixed names):**

* `uc_explist`
* `astar_explist_manhattan`
* `astar_explist_misplacedtiles`

## Expected Output

Console rows with columns:
`ALGORITHM, INIT_STATE, GOAL_STATE, PATH_LENGTH, STATE_EXPANSIONS, MAX_QLENGTH, RUNNING_TIME, DELETIONS_MIDDLE_HEAP, LOCAL_LOOPS_AVOIDED, ATTEMPTED_REEXPANSIONS, PATH, COMMENTS`

For the **Excel submission**:

* File name: `results_<ID>.xlsx` (e.g., `results_23010093.xlsx`)
* **Worksheet name must be `results`**
* Columns must **exactly** match the template and order above.

## Repository Layout

```
.
├─ main.cpp
├─ algorithm.cpp / algorithm.h
├─ puzzle.cpp / puzzle.h
├─ makefile          # optional on macOS; required for marking
└─ (docs / results)  # optional: Excel + logs
```

## Notes

* Heuristics:

  * *Misplaced Tiles* (do **not** count tile 0)
  * *Manhattan Distance* (sum of |dx|+|dy| for tiles 1..8)
* UC and both A\* variants must return optimal path length; efficiency ranking typically:
  **A\*(Manhattan) ≤ A\*(Misplaced) ≪ UC** in expansions/time.

