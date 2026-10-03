"""Reproduce the architecture audit; does not edit production files.

Run with Python 3 after configuring/building build/audit.
Windows default compiler: C:/msys64/ucrt64/bin/g++.exe.
Pass --headers to compile each public DCM header in isolation.
"""
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "build" / "audit"
OUT.mkdir(parents=True, exist_ok=True)
COMPILER = os.environ.get("AUDIT_CXX", "C:/msys64/ucrt64/bin/g++.exe")
INCLUDE_DIRS = [ROOT / "headers"] + sorted(p for p in (ROOT / "headers").rglob("*") if p.is_dir())
INCLUDE_DIRS += [ROOT / "math" / "headers"] + sorted(p for p in (ROOT / "math" / "headers").rglob("*") if p.is_dir())
INCLUDE_DIRS += [ROOT / "build" / "_deps" / "eigen-src"]
FLAGS = ["-std=c++20"] + ["-I" + str(p) for p in INCLUDE_DIRS]
ENV = os.environ.copy()
ENV["PATH"] = str(Path(COMPILER).parent) + os.pathsep + ENV.get("PATH", "")

def tracked(directory):
    return subprocess.check_output(["git", "-C", str(directory), "ls-files"], text=True).splitlines()

rows = []
for prefix, directory in [("", ROOT), ("math/", ROOT / "math")]:
    for name in tracked(directory):
        path = directory / name
        if not path.is_file():
            continue
        data = path.read_bytes()
        text = data.decode("utf-8", errors="replace") if path.suffix.lower() != ".png" else ""
        rows.append({"file": prefix + name, "bytes": len(data), "lines": len(text.splitlines()),
                     "tests": len(re.findall(r"^TEST(?:_F|_P)?\s*\(", text, re.M)),
                     "includes": re.findall(r'^\s*#\s*include\s*"([^"]+)"', text, re.M)})
(OUT / "inventory.json").write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8")
inventory_lines = ["# Инвентаризация файлов OurPaintDCM", "", "Снимок отслеживаемых файлов на 1 октября 2026 года. Созданные файлы аудита и build/ исключены. Статус использования относится к текущему репозиторию; внешние потребители публичного API не обследованы.", "", "| Файл | Строк | Роль |", "|---|---:|---|"]
for row in rows:
    name = row["file"]
    role = "Конфигурация / документация"
    if name.startswith("tests/"): role = "Тест DCM"
    elif name.startswith("math/tests/"): role = "Тест математического подмодуля"
    elif name.startswith("src/") or name.startswith("headers/"): role = "Библиотека DCM"
    elif name.startswith("math/src/") or name.startswith("math/headers/"): role = "Математический подмодуль"
    elif name.startswith("app/"): role = "Отдельное консольное приложение"
    if name.endswith("Components.h"): role = "Нет ссылок; не компилируется отдельно"
    if name in ["src/requirements/Requirements.cpp", "headers/requirements/Requirements.h"]: role = "Старый API; вызывается тестами, основной solve() обходит его"
    if name.endswith("RequirementFunctionFactory.h") or name.endswith("RequirementFunctionFactory.cpp"): role = "Фабрика; вызовы только в тестах"
    if name.endswith("DragOptimizersBenchmarkGTEST.cpp"): role = "Исключён из сборки; отдельной цели нет"
    if name.endswith(".png"): role = "Учебные материалы; ссылок в коде/README нет"
    target = (ROOT / name).as_posix()
    inventory_lines.append(f"| [{name}]({target}) | {row['lines'] if not name.endswith('.png') else '—'} | {role} |")
(ROOT / "docs" / "audit" / "FILE_INVENTORY.md").write_text("\n".join(inventory_lines) + "\n", encoding="utf-8")
for prefix in ["", "math/"]:
    selected = [r for r in rows if r["file"].startswith("math/") == bool(prefix)]
    print(f"inventory.{prefix or 'dcm'}: files={len(selected)}, lines={sum(r['lines'] for r in selected)}, test_declarations={sum(r['tests'] for r in selected)}", flush=True)

exe = OUT / ("repro.exe" if os.name == "nt" else "repro")
command = [COMPILER, *FLAGS, str(ROOT / "docs" / "audit" / "repro.cpp"),
           str(OUT / "libOurPaintDCM.a"), str(OUT / "math" / "libMath.a"), "-o", str(exe)]
result = subprocess.run(command, text=True, capture_output=True, env=ENV)
(OUT / "repro-build.log").write_text(result.stdout + result.stderr, encoding="utf-8")
if result.returncode:
    print(result.stderr)
    sys.exit(result.returncode)
result = subprocess.run([str(exe)], text=True, capture_output=True, env=ENV)
(OUT / "repro-output.txt").write_text(result.stdout + result.stderr, encoding="utf-8")
print(result.stdout + result.stderr, flush=True)
if result.returncode:
    sys.exit(result.returncode)

for mode in ["sparse-copy", "sparse-move"]:
    result = subprocess.run([str(exe), mode], text=True, capture_output=True, env=ENV)
    evidence = result.stdout + result.stderr + f"exit_code={result.returncode}\n"
    (OUT / f"{mode}-output.txt").write_text(evidence, encoding="utf-8")
    print(f"mode={mode}\n{evidence}", flush=True)

if "--headers" in sys.argv:
    checks = []
    for path in sorted((ROOT / "headers").rglob("*.h")):
        result = subprocess.run([COMPILER, *FLAGS, "-fsyntax-only", "-x", "c++", "-"],
                                input=f'#include "{path.as_posix()}"\n', capture_output=True, text=True, env=ENV)
        checks.append({"header": path.relative_to(ROOT).as_posix(), "exit": result.returncode, "stderr": result.stderr})
        print(f"header: {path.relative_to(ROOT)}: {'FAIL' if result.returncode else 'OK'}", flush=True)
    (OUT / "header-checks.json").write_text(json.dumps(checks, indent=2), encoding="utf-8")
