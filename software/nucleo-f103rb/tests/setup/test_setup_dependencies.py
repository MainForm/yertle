"""Run in the build image: python3 -m unittest discover -s tests/setup -v."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "setup_dependencies", Path(__file__).resolve().parents[2] / "scripts" / "setup-dependencies.py")
setup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)


class DependencySetupTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        (self.root / "third_party").mkdir()
        self.dep = dict(name="example", path="third_party/example",
                        repository="https://github.com/example/project.git", commit="a" * 40)

    def load(self, dep=None, version=1):
        lock = self.root / "dependencies.lock.json"
        lock.write_text(json.dumps(dict(schema_version=version, dependencies=[dep or self.dep])), encoding="utf-8")
        return setup.load_dependencies(self.root, lock)

    def checkout(self):
        directory = self.root / self.dep["path"]
        (directory / ".git").mkdir(parents=True)
        return directory

    def fake_git(self, directory, *args):
        results = {("rev-parse", "--show-toplevel"): str(directory),
                   ("rev-parse", "HEAD"): self.dep["commit"],
                   ("remote", "get-url", "origin"): self.dep["repository"],
                   ("status", "--porcelain", "--untracked-files=all"): ""}
        return results[args]

    def test_full_commit_lock(self):
        self.assertEqual(self.load(), [self.dep])

    def test_invalid_manifest_fields(self):
        for field, value in [("commit", "main"), ("commit", "a" * 7),
                             ("repository", "ssh://github.com/example/project"),
                             ("path", "../../outside"), ("name", "../outside")]:
            with self.subTest(field=field, value=value):
                with self.assertRaises(ValueError):
                    self.load(dict(self.dep, **{field: value}))
        with self.assertRaises(ValueError):
            self.load(version=2)

    def test_cached_checkout_never_fetches(self):
        self.checkout()
        with patch.object(setup, "git", side_effect=self.fake_git) as command:
            setup.prepare_dependency(self.root, self.dep)
            self.assertEqual(command.call_count, 4)

    def test_wrong_revision_not_overwritten(self):
        directory = self.checkout()
        def git(path, *args):
            return "b" * 40 if args == ("rev-parse", "HEAD") else self.fake_git(path, *args)
        with patch.object(setup, "git", side_effect=git), self.assertRaisesRegex(RuntimeError, "Commit mismatch"):
            setup.prepare_dependency(self.root, self.dep)
        self.assertTrue((directory / ".git").exists())

    def test_dirty_checkout_not_overwritten(self):
        directory = self.checkout()
        local_file = directory / "local-work.txt"
        local_file.write_text("preserve me", encoding="utf-8")
        def git(path, *args):
            return "?? local-work.txt" if args[0] == "status" else self.fake_git(path, *args)
        with patch.object(setup, "git", side_effect=git), self.assertRaisesRegex(RuntimeError, "Local modifications"):
            setup.prepare_dependency(self.root, self.dep)
        self.assertEqual(local_file.read_text(encoding="utf-8"), "preserve me")

    def test_missing_verify_only_does_not_download(self):
        with patch.object(setup, "git") as command, self.assertRaisesRegex(RuntimeError, "Missing"):
            setup.prepare_dependency(self.root, self.dep, verify_only=True)
        command.assert_not_called()

    def test_failed_download_leaves_no_partial_final_checkout(self):
        with patch.object(setup, "git", side_effect=RuntimeError("network failure")), self.assertRaises(RuntimeError):
            setup.prepare_dependency(self.root, self.dep)
        self.assertFalse((self.root / self.dep["path"]).exists())
        self.assertEqual(list((self.root / "third_party").iterdir()), [])

    def test_symlink_is_rejected(self):
        (self.root / self.dep["path"]).symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError):
            self.load()
        with self.assertRaisesRegex(RuntimeError, "symlink"):
            setup.prepare_dependency(self.root, self.dep)


if __name__ == "__main__":
    unittest.main()
