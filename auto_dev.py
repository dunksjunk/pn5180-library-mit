#!/usr/bin/env python3
"""
auto_dev.py - closed-loop development harness for the PN5180 clean-room driver.

Roles
-----
  Author   : Claude Code, driven headless (`claude -p`), resuming one session
             across iterations so it keeps context. Edits files in place.
  Gates    : deterministic checks - arduino-cli compile of every example for
             every target board (-Wall), optional cppcheck, optional custom
             commands (host unit tests etc.).
  Reviewer : Sourcery. There is no local C++ CLI for Sourcery (the old
             `sourcery review` CLI is legacy and Python/JS-only), so two modes:
               inbox - you run Sourcery's IDE review ("working changes vs
                       HEAD") and paste the findings into
                       .autodev/sourcery/inbox.md; the harness feeds them in.
               pr    - push the work branch to a private GitHub/GitLab repo
                       with an open PR; the harness requests a Sourcery review
                       through the REST API and pulls the OPEN comments.

Loop
----
  1. run gates; ask Sourcery only when gates are clean
  2. hand the findings to Claude; Claude fixes them or *disputes* them with a
     reason (compile errors cannot be disputed). Disputes land in a ledger for
     you to adjudicate instead of ping-ponging forever
  3. commit a checkpoint on a local autodev/* branch
  4. stop when Claude reports the task done and nothing is open, or on
     stall / budget / iteration limit / Claude asking a question

Everything a run does is logged under .autodev/runs/<timestamp>/.

Usage
-----
  python auto_dev.py --check                         # gates only, no Claude
  python auto_dev.py --task "Implement SPI transport + readRegister/writeRegister"
  python auto_dev.py --task-file TASK.md --sourcery inbox --wait-sourcery
  python auto_dev.py --resume-last --task "continue"  # pick up the last session

Stdlib only. Python 3.11+.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent
STATE = ROOT / ".autodev"
SPECS = ROOT / ".specs"
SPEC_TEXT = SPECS / "text"

# ESP32 family first: classic (Xtensa LX6), S3 (Xtensa LX7, native USB), C3 (RISC-V).
DEFAULT_FQBNS = ["esp32:esp32:esp32", "esp32:esp32:esp32s3", "esp32:esp32:esp32c3"]
GH = shutil.which("gh") or r"C:\Program Files\GitHub CLI\gh.exe"  # winget installs here; PATH may lag
COMMIT_TRAILER = "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"

# What the headless author may do. No web access (clean room), no git writes
# (the harness owns checkpoints), no arbitrary shell.
CLAUDE_ALLOWED_TOOLS = [
    "Read", "Edit", "Write", "Glob", "Grep",
    "Bash(arduino-cli compile:*)",
    "Bash(cppcheck:*)",
    "Bash(git status:*)", "Bash(git diff:*)", "Bash(git log:*)",
]
CLAUDE_DISALLOWED_TOOLS = ["WebFetch", "WebSearch", "Agent"]

SYSTEM_PROMPT = """\
You are the author in an automated closed-loop harness building a PN5180 NFC
frontend driver for Arduino / ESP32, released under the MIT license.

CLEAN-ROOM RULES (non-negotiable):
- The only technical sources are the NXP documents in .specs/ (PDFs, with
  extracted text in .specs/text/*.txt - grep those). Use nothing else.
- Do not recall, reproduce, paraphrase or imitate any existing PN5180 library
  (e.g. ATrappmann/PN5180-Library or its forks, NXP NFC Reader Library). If you
  notice you are reconstructing one from memory, stop and derive from the
  datasheet instead.
- Every command code, register address, bit field, EEPROM address and timing
  constant gets a comment citing its source, e.g. `// DS 11.4.3.3` or
  `// AN12650 3.2`. Never invent values; if the spec is silent, say so and
  report it under "questions".

TARGETS:
- Primary: ESP32 family on Arduino-ESP32 (classic, S3, C3 - all compiled by
  the harness). Test bed: ESP32-D0WD-V3 dev board (WROOM, 4 MB, esp32:esp32:esp32)
  + generic PN5180 module. Pins: SCK 18, MISO 19, MOSI 23, NSS 16, BUSY 5,
  RST 17, IRQ not connected (IRQ must be optional; poll IRQ_STATUS instead).
  NSS is a plain GPIO driven by the driver - Arduino's default SS is GPIO5,
  which is BUSY here, so never let SPI.begin() claim SS. Examples default to
  this pin map but every pin is configurable.
- Next: Raspberry Pi on Linux (first board: Pi Zero 2 W) via /dev/spidev and
  the GPIO character device (libgpiod). Not compiled by the harness yet, but
  design for it now.

ENGINEERING RULES:
- Arduino library layout: library.properties, src/, examples/<Name>/<Name>.ino.
  A CMake build for Linux will be added later; don't make it impossible.
- Core protocol code is portable C++17 that never includes Arduino.h or any
  OS header. Platform code lives in separate HAL files (e.g. an Arduino HAL
  and later a Linux HAL) behind a small interface: SPI transfer, NSS, BUSY,
  RST, optional IRQ, millisecond clock/delay.
- No exceptions, RTTI or heap allocation in the core; fixed-size buffers;
  every wait on BUSY or IRQ has a timeout.
- Keep the core unit-testable on a PC with a fake HAL.
- Every public function returns an explicit status; never silently ignore a
  PN5180 error or a BUSY timeout.
- Each source file starts with an SPDX-License-Identifier: MIT header.

HOW THIS LOOP WORKS:
- Each turn gives you findings from compilers/analyzers and from Sourcery,
  each tagged with an id like [a1b2c3d4]. Fix them. If you believe a finding
  is wrong (warnings and review comments only - never compile errors), you
  may dispute it with a concrete, spec-grounded reason; a human adjudicates.
- When there are no findings, keep advancing the task.
- Do not run git commit/add/checkout; the harness checkpoints for you.
- End every turn with the structured result. Set status "done" only when the
  whole task is complete, "blocked" when you need a human decision.
"""

RESULT_SCHEMA = {
    "type": "object",
    "properties": {
        "status": {"type": "string", "enum": ["progress", "done", "blocked"]},
        "summary": {"type": "string"},
        "disputed": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {"id": {"type": "string"}, "reason": {"type": "string"}},
                "required": ["id", "reason"],
            },
        },
        "questions": {"type": "array", "items": {"type": "string"}},
    },
    "required": ["status", "summary", "disputed", "questions"],
}


# --------------------------------------------------------------------------- #
# Findings
# --------------------------------------------------------------------------- #
@dataclass
class Finding:
    source: str          # "compile:<fqbn>", "cppcheck", "gate:<name>", "sourcery"
    severity: str        # "error" | "warning" | "info"
    message: str
    path: str = ""
    line: int = 0
    rule: str = ""
    fingerprint: str = field(default="", init=False)

    def __post_init__(self) -> None:
        # Line numbers deliberately excluded so a finding keeps its identity
        # while surrounding code moves; digits collapsed for the same reason.
        norm = re.sub(r"\d+", "#", self.message.strip().lower())[:300]
        src = self.source.split(":")[0]
        key = f"{src}|{self.path}|{self.rule}|{norm}"
        self.fingerprint = hashlib.sha1(key.encode()).hexdigest()[:8]

    @property
    def disputable(self) -> bool:
        return self.severity != "error"

    def render(self) -> str:
        loc = f"{self.path}:{self.line}" if self.path else "(project)"
        rule = f" ({self.rule})" if self.rule else ""
        return f"[{self.fingerprint}] {self.source} {self.severity.upper()} {loc}{rule}\n    {self.message.strip()}"


def dedupe(findings: list[Finding]) -> list[Finding]:
    """Same warning on three boards -> one finding (keeps the first source)."""
    seen: dict[str, Finding] = {}
    for f in findings:
        seen.setdefault(f.fingerprint, f)
    return list(seen.values())


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def run(cmd: list[str], *, cwd: Path = ROOT, timeout: int = 900, input: str | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd, cwd=cwd, input=input, capture_output=True, text=True,
        encoding="utf-8", errors="replace", timeout=timeout,
    )


def git(*args: str) -> str:
    r = run(["git", *args])
    if r.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)} failed:\n{r.stderr.strip()}")
    return r.stdout.strip()


def rel(path: str) -> str | None:
    """Project-relative posix path, or None if the file is outside the project."""
    try:
        return Path(path).resolve().relative_to(ROOT).as_posix()
    except (ValueError, OSError):
        return None


def log(msg: str) -> None:
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


# --------------------------------------------------------------------------- #
# Gates
# --------------------------------------------------------------------------- #
GCC_DIAG = re.compile(
    r"^(?P<path>(?:[A-Za-z]:)?[^:\n]+?):(?P<line>\d+):(?:(?P<col>\d+):)?\s+"
    r"(?P<sev>fatal error|error|warning):\s+(?P<msg>.*?)(?:\s+\[(?P<rule>-W[^\]]+)\])?$",
    re.MULTILINE,
)


def find_examples() -> list[Path]:
    ex = ROOT / "examples"
    if not ex.is_dir():
        return []
    return sorted(d for d in ex.iterdir() if (d / f"{d.name}.ino").is_file())


def gate_compile(fqbns: list[str]) -> list[Finding]:
    examples = find_examples()
    if not examples:
        return [Finding("compile", "error",
                        "No examples/<Name>/<Name>.ino found. Create at least one minimal example "
                        "sketch that exercises the library so every target board can be compile-checked.")]
    out: list[Finding] = []
    for fqbn in fqbns:
        for ex in examples:
            build = ROOT / "build" / fqbn.replace(":", "_") / ex.name
            log(f"  compile {ex.name} for {fqbn}")
            r = run(["arduino-cli", "compile", "--fqbn", fqbn, "--warnings", "all",
                     "--library", str(ROOT), "--build-path", str(build), str(ex)], timeout=1800)
            text = r.stdout + "\n" + r.stderr
            this: list[Finding] = []
            for m in GCC_DIAG.finditer(text):
                path = rel(m["path"])
                sev = "error" if "error" in m["sev"] else "warning"
                if path is None and sev == "warning":
                    continue  # warnings from cores / third-party code are not ours
                this.append(Finding(f"compile:{fqbn}", sev, m["msg"], path or m["path"],
                                    int(m["line"]), m["rule"] or ""))
            out += this
            if r.returncode != 0 and not any(f.severity == "error" for f in this):
                tail = "\n".join(text.strip().splitlines()[-25:])
                out.append(Finding(f"compile:{fqbn}", "error",
                                   f"Build of {ex.name} failed without a parsable diagnostic:\n{tail}"))
    return out


def gate_cppcheck() -> list[Finding]:
    if not shutil.which("cppcheck") or not (ROOT / "src").is_dir():
        return []
    log("  cppcheck src/")
    r = run(["cppcheck", "--enable=warning,style,performance,portability", "--std=c++17",
             "--inline-suppr", "--quiet", "--suppress=missingIncludeSystem",
             "--template={file}:{line}:{column}: {severity}: {message} [{id}]", "src"])
    out = []
    for line in r.stderr.splitlines():
        m = re.match(r"^(.+?):(\d+):\d+: (\w+): (.*) \[(\w+)\]$", line)
        if m:
            sev = "error" if m[3] == "error" else "warning"
            out.append(Finding("cppcheck", sev, m[4], rel(m[1]) or m[1], int(m[2]), m[5]))
    return out


def gate_custom(commands: list[str]) -> list[Finding]:
    out = []
    for i, cmd in enumerate(commands, 1):
        log(f"  gate: {cmd}")
        r = subprocess.run(cmd, cwd=ROOT, shell=True, capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=1800)
        if r.returncode != 0:
            tail = "\n".join((r.stdout + r.stderr).strip().splitlines()[-40:])
            out.append(Finding(f"gate:{i}", "error", f"`{cmd}` exited {r.returncode}:\n{tail}"))
    return out


# --------------------------------------------------------------------------- #
# Sourcery
# --------------------------------------------------------------------------- #
class SourceryOff:
    name = "off"

    def findings(self, iteration: int) -> list[Finding] | None:
        return []  # [] = clean, None = no verdict yet

    def consumed(self) -> None:
        pass


class SourceryInbox:
    """You run the Sourcery IDE review and paste results into inbox.md.

    Write `LGTM` alone in the file to signal Sourcery had nothing to say.
    Each bullet / numbered item / heading-delimited block becomes one finding;
    a `path:line` anywhere in it is picked up as the location.
    """
    name = "inbox"
    LOC = re.compile(r"((?:src|examples|test)/[\w./-]+\.(?:hpp|h|cpp|c|ino))\b(?::(\d+))?")

    def __init__(self, wait: bool):
        self.dir = STATE / "sourcery"
        self.inbox = self.dir / "inbox.md"
        self.wait = wait
        self.dir.mkdir(parents=True, exist_ok=True)
        self.inbox.touch(exist_ok=True)

    def _parse(self, text: str) -> list[Finding]:
        blocks = re.split(r"\n(?=\s*(?:[-*]|\d+[.)]|#{1,4})\s)", "\n" + text.strip())
        out = []
        for b in (b.strip() for b in blocks):
            if not b:
                continue
            m = self.LOC.search(b)
            out.append(Finding("sourcery", "warning", b, m[1] if m else "", int(m[2] or 0) if m else 0))
        return out

    def findings(self, iteration: int) -> list[Finding] | None:
        text = self.inbox.read_text(encoding="utf-8").strip()
        if not text and self.wait:
            log(f"Gates are clean. Run Sourcery's IDE review (working changes vs the branch base),\n"
                f"           paste findings into {self.inbox.relative_to(ROOT)} - or write LGTM. Waiting...")
            while not text:
                time.sleep(5)
                text = self.inbox.read_text(encoding="utf-8").strip()
        if not text:
            return None
        if text.upper() == "LGTM":
            self.consumed()
            return []
        return self._parse(text)

    def consumed(self) -> None:
        if self.inbox.read_text(encoding="utf-8").strip():
            done = self.dir / "processed"
            done.mkdir(exist_ok=True)
            shutil.move(self.inbox, done / f"{datetime.now():%Y%m%d-%H%M%S}.md")
            self.inbox.touch()


class SourceryPR:
    """Sourcery REST API against an open PR on a private remote.

    Needs SOURCERY_API_KEY and the Sourcery GitHub app installed on the repo.
    --sourcery-repo / --sourcery-pr are optional: by default the repo comes
    from `gh repo view` and the PR is the open PR for the current branch,
    created (ready for review, not draft - Sourcery skips drafts) if missing.
    The harness pushes the work branch before each review request.
    """
    name = "pr"
    API = "https://api.sourcery.ai/api/v1/pull-request-reviews"

    def __init__(self, repo: str | None, pr: int | None, remote: str, base: str, timeout_s: int = 1200):
        self.key = os.environ.get("SOURCERY_API_KEY")
        if not self.key:
            sys.exit("--sourcery pr needs the SOURCERY_API_KEY environment variable")
        if not repo:
            repo = run([GH, "repo", "view", "--json", "nameWithOwner", "-q", ".nameWithOwner"]).stdout.strip()
            if not repo:
                sys.exit("Could not determine the GitHub repo; pass --sourcery-repo owner/name")
        self.repo, self.pr, self.remote, self.base, self.timeout_s = repo, pr, remote, base, timeout_s

    def _ensure_pr(self, branch: str) -> int:
        if self.pr:
            return self.pr
        r = run([GH, "pr", "view", branch, "--json", "number,state", "-q", "select(.state==\"OPEN\") | .number"])
        if r.stdout.strip().isdigit():
            self.pr = int(r.stdout.strip())
        else:
            log(f"  opening PR {branch} -> {self.base}")
            r = run([GH, "pr", "create", "--base", self.base, "--head", branch,
                     "--title", f"autodev: {branch}",
                     "--body", "Automated auto_dev.py work branch, reviewed by Sourcery each iteration.\n\n"
                               "🤖 Generated with [Claude Code](https://claude.com/claude-code)"])
            if r.returncode != 0:
                raise RuntimeError(f"gh pr create failed: {r.stderr.strip()}")
            self.pr = int(r.stdout.strip().rstrip("/").rsplit("/", 1)[-1])
        log(f"  using PR #{self.pr} on {self.repo}")
        return self.pr

    def _call(self, method: str, url: str, body: dict | None = None) -> dict:
        req = urllib.request.Request(
            url, method=method, data=json.dumps(body).encode() if body else None,
            headers={"Authorization": f"Bearer {self.key}", "Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=60) as resp:
                return json.load(resp)
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"Sourcery API {method} {e.code}: {e.read().decode(errors='replace')}") from e

    def findings(self, iteration: int) -> list[Finding] | None:
        branch = git("rev-parse", "--abbrev-ref", "HEAD")
        log(f"  push {branch} -> {self.remote}")
        git("push", "-u", self.remote, branch)
        self._ensure_pr(branch)
        req = self._call("POST", self.API, {"repository": self.repo, "pull_request": self.pr})
        rid, deadline = req["id"], time.time() + self.timeout_s
        query = f"{self.API}?{urllib.parse.urlencode({'repository': self.repo, 'pull_request': self.pr})}"
        log(f"  Sourcery review request {rid} queued")
        while time.time() < deadline:
            time.sleep(20)
            data = self._call("GET", query)
            mine = next((r for r in data.get("requests", []) if r["id"] == rid), None)
            status = (mine or {}).get("status")
            if status in ("REFUSED", "FAILED"):
                raise RuntimeError(f"Sourcery review {status}: {mine.get('refusal_reason')}")
            if status == "DONE":
                return self._open_comments(data)
        raise RuntimeError("Timed out waiting for Sourcery review")

    @staticmethod
    def _open_comments(data: dict) -> list[Finding]:
        out = []
        for review in data.get("reviews", []):
            if review.get("dismissed_at"):
                continue
            for c in review.get("comments", []):
                if c.get("status") != "OPEN":
                    continue
                sev = "error" if c.get("is_blocking") else "warning"
                out.append(Finding("sourcery", sev, c.get("body", ""), c.get("path", ""),
                                   c.get("start_line") or 0, c.get("rule_id") or c.get("category") or ""))
        return out

    def consumed(self) -> None:
        pass


# --------------------------------------------------------------------------- #
# Claude (author)
# --------------------------------------------------------------------------- #
@dataclass
class ClaudeTurn:
    status: str
    summary: str
    disputed: list[dict]
    questions: list[str]
    session_id: str | None
    cost_usd: float
    raw: dict


class Claude:
    def __init__(self, model: str | None, turn_budget: float, session_id: str | None):
        exe = shutil.which("claude")
        if not exe:
            sys.exit("Claude Code CLI (`claude`) not found on PATH")
        self.exe, self.model, self.turn_budget, self.session_id = exe, model, turn_budget, session_id

    def turn(self, prompt: str) -> ClaudeTurn:
        cmd = [self.exe, "-p", "--output-format", "json",
               "--permission-mode", "acceptEdits",
               "--append-system-prompt", SYSTEM_PROMPT,
               "--json-schema", json.dumps(RESULT_SCHEMA),
               "--max-budget-usd", f"{self.turn_budget:.2f}",
               "--strict-mcp-config",  # no MCP servers: keeps the author offline
               "--allowedTools", *CLAUDE_ALLOWED_TOOLS,
               "--disallowedTools", *CLAUDE_DISALLOWED_TOOLS]
        if self.model:
            cmd += ["--model", self.model]
        if self.session_id:
            cmd += ["--resume", self.session_id]
        # Prompt goes over stdin: findings can exceed the Windows command-line limit.
        r = run(cmd, input=prompt, timeout=3600)
        try:
            data = json.loads(r.stdout)
        except json.JSONDecodeError:
            raise RuntimeError(f"claude returned non-JSON (exit {r.returncode}):\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
        self.session_id = data.get("session_id") or self.session_id
        result = data.get("structured_output")
        if result is None:
            try:
                result = json.loads(data.get("result") or "")
            except (json.JSONDecodeError, TypeError):
                result = {"status": "blocked" if data.get("is_error") else "progress",
                          "summary": (data.get("result") or "")[:2000], "disputed": [], "questions": []}
        return ClaudeTurn(result.get("status", "progress"), result.get("summary", ""),
                          result.get("disputed", []), result.get("questions", []),
                          self.session_id, float(data.get("total_cost_usd") or 0), data)


def build_prompt(iteration: int, task: str, findings: list[Finding], n_disputed: int,
                 sourcery_pending: bool, first: bool) -> str:
    parts = [f"# Iteration {iteration}", f"## Task\n{task}"]
    if first:
        specs = sorted(p.relative_to(ROOT).as_posix() for p in SPEC_TEXT.glob("*.txt"))
        parts.append(
            "## Context\n"
            f"- Spec text (grep these, cite sections): {', '.join(specs) or 'none extracted - read the PDFs in .specs/'}\n"
            "- Read the existing tree before writing; the project may already be partly built.")
    if findings:
        parts.append(f"## Findings to resolve ({len(findings)})\n" + "\n".join(f.render() for f in findings))
        parts.append("Fix each finding, or dispute non-error findings by id with a spec-grounded reason.")
    else:
        parts.append("## Findings\nNone open. Continue the task.")
        if sourcery_pending:
            parts.append("(Sourcery has not reviewed the current state yet.)")
    if n_disputed:
        parts.append(f"({n_disputed} previously disputed finding(s) are parked for human review and hidden.)")
    return "\n\n".join(parts)


# --------------------------------------------------------------------------- #
# Ledger / run log
# --------------------------------------------------------------------------- #
class Ledger:
    """Persistent record of disputed findings, keyed by fingerprint."""

    def __init__(self) -> None:
        self.path = STATE / "ledger.json"
        self.data: dict = json.loads(self.path.read_text("utf-8")) if self.path.exists() else {}

    def disputed(self) -> set[str]:
        return {fp for fp, v in self.data.items() if v.get("status") == "disputed"}

    def record(self, disputes: list[dict], findings: list[Finding], iteration: int) -> list[str]:
        by_id = {f.fingerprint: f for f in findings}
        rejected = []
        for d in disputes:
            f = by_id.get(d.get("id", "").strip("[]"))
            if f is None or not f.disputable:
                rejected.append(d.get("id", "?"))
                continue
            self.data[f.fingerprint] = {"status": "disputed", "reason": d.get("reason", ""),
                                        "iteration": iteration, "finding": asdict(f),
                                        "at": datetime.now().isoformat(timespec="seconds")}
        self.path.write_text(json.dumps(self.data, indent=2), "utf-8")
        return rejected


# --------------------------------------------------------------------------- #
# Setup
# --------------------------------------------------------------------------- #
def find_pdftotext() -> str | None:
    # Git for Windows / MSYS2 ship poppler's pdftotext but don't put it on the
    # PowerShell/cmd PATH, so check their usual locations too.
    candidates = [shutil.which("pdftotext"),
                  r"C:\Program Files\Git\ucrt64\bin\pdftotext.exe",
                  r"C:\Program Files\Git\mingw64\bin\pdftotext.exe",
                  r"C:\msys64\ucrt64\bin\pdftotext.exe",
                  r"C:\msys64\mingw64\bin\pdftotext.exe"]
    return next((c for c in candidates if c and Path(c).is_file()), None)


def extract_spec_text() -> None:
    pdfs = list(SPECS.glob("*.pdf"))
    stale = [p for p in pdfs if not (SPEC_TEXT / f"{p.stem}.txt").exists()
             or (SPEC_TEXT / f"{p.stem}.txt").stat().st_mtime < p.stat().st_mtime]
    if not stale:
        return
    exe = find_pdftotext()
    if not exe:
        sys.exit("pdftotext not found and .specs/text/ is missing or stale. Install poppler "
                 "(or Git for Windows' ucrt64 tools) - the author must not work without the spec text.")
    SPEC_TEXT.mkdir(parents=True, exist_ok=True)
    for pdf in stale:
        txt = SPEC_TEXT / f"{pdf.stem}.txt"
        log(f"extracting {pdf.name} -> {txt.relative_to(ROOT)}")
        run([exe, "-layout", str(pdf), str(txt)])


def preflight(args: argparse.Namespace) -> None:
    if not shutil.which("arduino-cli"):
        sys.exit("arduino-cli not found on PATH")
    if run(["git", "rev-parse", "--is-inside-work-tree"]).returncode != 0:
        sys.exit("Not a git repository. Run `git init` first (no remote needed); "
                 "the harness checkpoints every iteration as a commit.")
    STATE.mkdir(exist_ok=True)
    extract_spec_text()
    if args.check:
        return
    if git("status", "--porcelain") and not args.allow_dirty:
        sys.exit("Working tree is dirty. Commit/stash first, or pass --allow-dirty.")
    branch = git("rev-parse", "--abbrev-ref", "HEAD")
    if not branch.startswith("autodev/") and not args.stay_on_branch:
        new = f"autodev/{datetime.now():%Y%m%d-%H%M}"
        log(f"creating work branch {new} (from {branch})")
        git("checkout", "-b", new)


def checkpoint(iteration: int, summary: str) -> bool:
    if not git("status", "--porcelain"):
        return False
    git("add", "-A")
    title = re.sub(r"\s+", " ", summary).strip()[:60] or "update"
    git("commit", "-q", "-m", f"autodev iter {iteration}: {title}\n\n{summary.strip()}\n\n{COMMIT_TRAILER}")
    return True


# --------------------------------------------------------------------------- #
# Main loop
# --------------------------------------------------------------------------- #
def collect(args: argparse.Namespace, sourcery, ledger: Ledger, iteration: int) -> tuple[list[Finding], bool, bool]:
    """Returns (active findings, sourcery_pending, sourcery_delivered)."""
    log("running gates")
    found = gate_compile(args.fqbn) + gate_cppcheck() + gate_custom(args.gate)
    if not args.strict_warnings:
        found = [f for f in found if not (f.source.startswith("compile") and f.severity == "warning"
                                          and f.rule in ("-Wunused-parameter",))]
    pending = delivered = False
    if sourcery is not None and not any(f.severity == "error" for f in found):
        log(f"asking Sourcery ({sourcery.name})")
        try:
            s = sourcery.findings(iteration)
        except (RuntimeError, OSError) as e:
            # Network/auth/API trouble shouldn't kill a run that has paid for
            # Claude turns: log it, treat the review as pending, keep going.
            log(f"Sourcery unavailable this iteration: {e}")
            s = None
        if s is None:
            pending = True
        else:
            delivered = bool(s)
            found += s
    disputed = ledger.disputed()
    return [f for f in dedupe(found) if f.fingerprint not in disputed], pending, delivered


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", help="what Claude should accomplish this run")
    ap.add_argument("--task-file", type=Path, help="read the task from a file instead")
    ap.add_argument("--check", action="store_true", help="run gates once, print findings, no Claude")
    ap.add_argument("--fqbn", action="append", help=f"target board (repeatable). default: {DEFAULT_FQBNS}")
    ap.add_argument("--gate", action="append", default=[], help="extra shell command that must exit 0 (repeatable)")
    ap.add_argument("--strict-warnings", action="store_true", help="also feed -Wunused-parameter etc.")
    ap.add_argument("--sourcery", choices=["inbox", "pr", "off"], default="inbox")
    ap.add_argument("--wait-sourcery", action="store_true", help="inbox mode: block until you paste a review")
    ap.add_argument("--sourcery-repo", help="pr mode: owner/name")
    ap.add_argument("--sourcery-pr", type=int, help="pr mode: PR number")
    ap.add_argument("--remote", default="origin", help="pr mode: remote to push to")
    ap.add_argument("--base", default="main", help="pr mode: base branch for an auto-created PR")
    ap.add_argument("--model", help="Claude model for the author (default: Claude Code's default)")
    ap.add_argument("--max-iter", type=int, default=8)
    ap.add_argument("--turn-budget", type=float, default=5.0, help="USD cap per Claude turn")
    ap.add_argument("--run-budget", type=float, default=25.0, help="USD cap for the whole run")
    ap.add_argument("--resume-last", action="store_true", help="resume the Claude session from the last run")
    ap.add_argument("--allow-dirty", action="store_true")
    ap.add_argument("--stay-on-branch", action="store_true", help="don't create an autodev/* branch")
    args = ap.parse_args()
    args.fqbn = args.fqbn or DEFAULT_FQBNS

    if args.sourcery == "pr" and args.sourcery_pr:
        args.stay_on_branch = True  # an explicit PR is tied to the branch it was opened from
    preflight(args)
    ledger = Ledger()

    if args.check:
        findings, _, _ = collect(args, None, ledger, 0)
        print("\n".join(f.render() for f in findings) or "clean")
        return 1 if findings else 0

    if args.sourcery == "pr":
        sourcery = SourceryPR(args.sourcery_repo, args.sourcery_pr, args.remote, args.base)
    elif args.sourcery == "inbox":
        sourcery = SourceryInbox(args.wait_sourcery)
    else:
        sourcery = SourceryOff()

    task = args.task_file.read_text("utf-8") if args.task_file else args.task
    if not task:
        sys.exit("--task or --task-file is required (or use --check)")

    state_file = STATE / "last_session.json"
    prior = json.loads(state_file.read_text("utf-8")) if args.resume_last and state_file.exists() else {}
    claude = Claude(args.model, args.turn_budget, prior.get("session_id"))
    last_status = prior.get("status", "progress")

    run_dir = STATE / "runs" / f"{datetime.now():%Y%m%d-%H%M%S}"
    run_dir.mkdir(parents=True)
    spent, stall, prev_fps = 0.0, 0, None
    outcome, questions = "max iterations reached", []

    for it in range(1, args.max_iter + 1):
        log(f"=== iteration {it}/{args.max_iter} (spent ${spent:.2f}) ===")
        findings, pending, delivered = collect(args, sourcery, ledger, it)
        fps = {f.fingerprint for f in findings}
        log(f"{len(findings)} open finding(s)" + (" - Sourcery verdict pending" if pending else ""))

        if not findings and last_status == "done":
            if pending:
                outcome = "task done, gates clean - awaiting Sourcery review (paste into inbox and re-run)"
            else:
                outcome = "success: task done, gates clean, Sourcery clean"
            break

        prompt = build_prompt(it, task, findings, len(ledger.disputed()), pending, first=claude.session_id is None)
        log("Claude working...")
        last = claude.turn(prompt)
        last_status = last.status
        spent += last.cost_usd
        state_file.write_text(json.dumps({"session_id": last.session_id, "status": last.status}), "utf-8")
        if delivered:
            sourcery.consumed()  # archive the inbox only once Claude has actually seen it

        rejected = ledger.record(last.disputed, findings, it)
        committed = checkpoint(it, last.summary)
        (run_dir / f"iter_{it:02d}.json").write_text(json.dumps({
            "prompt": prompt, "findings": [asdict(f) for f in findings], "status": last.status,
            "summary": last.summary, "disputed": last.disputed, "rejected_disputes": rejected,
            "questions": last.questions, "cost_usd": last.cost_usd, "committed": committed,
        }, indent=2), "utf-8")
        log(f"Claude: {last.status} (${last.cost_usd:.2f}) - {last.summary[:200]}")
        if rejected:
            log(f"rejected disputes (errors/unknown ids): {rejected}")

        if last.status == "blocked":
            outcome, questions = "blocked: Claude needs a decision", last.questions
            break
        if spent >= args.run_budget:
            outcome = f"run budget ${args.run_budget:.2f} exhausted"
            break
        stall = stall + 1 if (not committed and fps == prev_fps) else 0
        if stall >= 2:
            outcome = "stalled: no file changes and same findings twice in a row"
            break
        prev_fps = fps

    report = [f"# auto_dev run {run_dir.name}", f"Outcome: {outcome}", f"Spent: ${spent:.2f}",
              f"Branch: {git('rev-parse', '--abbrev-ref', 'HEAD')}", f"Claude session: {claude.session_id}"]
    if questions:
        report += ["", "## Questions from Claude", *[f"- {q}" for q in questions]]
    parked = ledger.disputed()
    if parked:
        report += ["", "## Disputed findings awaiting your call (.autodev/ledger.json)"]
        report += [f"- [{fp}] {ledger.data[fp]['finding']['message'][:120]!r} -> {ledger.data[fp]['reason']}"
                   for fp in sorted(parked)]
    (run_dir / "report.md").write_text("\n".join(report) + "\n", "utf-8")
    print("\n" + "\n".join(report))
    return 0 if outcome.startswith("success") else 1


if __name__ == "__main__":
    sys.exit(main())
