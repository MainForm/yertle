#!/usr/bin/env python3
"""Fetch exact source commits, without requiring the enclosing repository's .git.

Runs in the Linux Docker environment. Downloads are ignored local checkouts,
not vendored source or gitlinks. Existing mismatched/dirty trees are never reset.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


def load_dependencies(root, lock_file):
    if (root / "third_party").is_symlink():
        raise ValueError("third_party must not be a symlink outside the workspace")
    lock = json.loads(lock_file.read_text(encoding="utf-8"))
    if lock.get("schema_version") != 1 or not lock.get("dependencies"):
        raise ValueError("Expected schema_version=1 and a nonempty dependency list")
    dependencies = lock["dependencies"]
    seen = set()
    for dep in dependencies:
        name = dep["name"]
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name):
            raise ValueError(f"Unsafe dependency name: {name!r}")
        if dep["path"] != f"third_party/{name}" or name in seen:
            raise ValueError(f"Invalid/duplicate dependency path: {dep['path']!r}")
        seen.add(name)
        if (root / dep["path"]).resolve().parent != (root / "third_party").resolve():
            raise ValueError(f"Dependency escapes third_party: {name}")
        if not re.fullmatch(r"[0-9a-f]{40}", dep["commit"]):
            raise ValueError(f"Dependency needs a full lowercase commit SHA: {name}")
        if not re.fullmatch(r"https://github\.com/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+(?:\.git)?", dep["repository"]):
            raise ValueError(f"Expected a public HTTPS GitHub repository: {name}")
    return dependencies


def git(directory, *args):
    # Scope the ownership exception to this exact checkout, not every Git repo.
    command = ["git", "-c", f"safe.directory={directory}", "-c", "core.autocrlf=false",
               "-C", str(directory), *args]
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0")
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, env=env, timeout=600)
    if result.returncode:
        raise RuntimeError(f"git {' '.join(args)} failed in {directory}:\n{result.stderr.strip()}")
    return result.stdout.strip()


def verify_checkout(directory, dep):
    if directory.is_symlink() or not (directory / ".git").is_dir():
        raise RuntimeError(f"Refusing an unmanaged checkout at {directory}; no files were overwritten")
    if Path(git(directory, "rev-parse", "--show-toplevel")).resolve() != directory.resolve():
        raise RuntimeError(f"Unexpected Git root for {directory}")
    if git(directory, "remote", "get-url", "origin") != dep["repository"]:
        raise RuntimeError(f"Origin URL mismatch in {directory}; refusing to overwrite")
    actual = git(directory, "rev-parse", "HEAD")
    if actual != dep["commit"]:
        raise RuntimeError(f"Commit mismatch in {directory}: expected {dep['commit']}, got {actual}; "
                           "preserve local work and move the checkout aside before re-running setup")
    if git(directory, "status", "--porcelain", "--untracked-files=all"):
        raise RuntimeError(f"Local modifications in {directory}; refusing to overwrite or build")


def prepare_dependency(root, dep, verify_only=False):
    destination = root / dep["path"]
    if destination.is_symlink():
        raise RuntimeError(f"Refusing symlink dependency: {destination}")
    if destination.exists():
        if not destination.is_dir():
            raise RuntimeError(f"Dependency path is not a directory: {destination}")
        if any(destination.iterdir()):
            verify_checkout(destination, dep)
            print(f"CACHED {dep['name']} @ {dep['commit']}", flush=True)
            return
    if verify_only:
        raise RuntimeError(f"Missing {dep['name']}; run the Compose setup service")
    print(f"FETCH {dep['name']} @ {dep['commit']}", flush=True)
    # Only this newly created temporary directory is cleaned on failure.
    # Never delete/reset a user's pre-existing dependency checkout.
    with tempfile.TemporaryDirectory(prefix=f".setup-{dep['name']}-", dir=root / "third_party") as temporary:
        checkout = Path(temporary) / "checkout"
        checkout.mkdir()
        git(checkout, "init", "--quiet")
        git(checkout, "remote", "add", "origin", dep["repository"])
        git(checkout, "fetch", "--depth", "1", "origin", dep["commit"])
        git(checkout, "-c", "advice.detachedHead=false", "checkout", "--detach", "FETCH_HEAD")
        verify_checkout(checkout, dep)
        if destination.exists():
            # Empty directories left by gitlink checkout/import are harmless.
            # rmdir refuses nonempty targets; no recursive deletion is used.
            destination.rmdir()
        checkout.rename(destination)
    print(f"READY {dep['name']}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-only", action="store_true", help="No downloads; validate existing checkouts")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    dependencies = load_dependencies(root, root / "dependencies.lock.json")
    (root / "build").mkdir(exist_ok=True)
    (root / "third_party").mkdir(exist_ok=True)
    # Serializes setup invocations from 'setup' and 'f103-build'; a crash
    # releases the OS lock, so no stale 'in progress' flag needs deletion.
    with (root / "build" / ".dependency-setup.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        for dep in dependencies:
            prepare_dependency(root, dep, args.verify_only)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError, OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"DEPENDENCY ERROR: {error}", file=sys.stderr)
        sys.exit(1)
