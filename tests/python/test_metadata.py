"""Verify imports, package identity, and the editable development installation."""

import importlib
import json
from importlib.metadata import distribution
from pathlib import Path
from urllib.parse import unquote, urlparse

from aerotwin_experiments import package_name


def test_package_imports() -> None:
    package = importlib.import_module("aerotwin_experiments")
    assert callable(package.package_name)


def test_package_name() -> None:
    assert package_name() == "aerotwin_experiments"


def test_editable_installation() -> None:
    installed = distribution("aerotwin-experiments")
    direct_url = installed.read_text("direct_url.json")
    assert direct_url is not None, "Install the project with pip install -e '.[dev]'"
    metadata = json.loads(direct_url)
    assert metadata["dir_info"]["editable"] is True

    repository = Path(__file__).resolve().parents[2]
    source_url = urlparse(metadata["url"])
    assert source_url.scheme == "file"
    assert Path(unquote(source_url.path)).resolve() == repository

    package = importlib.import_module("aerotwin_experiments")
    expected = repository / "experiments/python/src/aerotwin_experiments/__init__.py"
    assert Path(package.__file__).resolve() == expected
