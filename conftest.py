"""Pytest configuration for the twin-fidelity harness.

Placing this at the repo root makes the ``harness`` package importable in
tests (pytest inserts the rootdir on sys.path) and provides a fixture that
guarantees the working directory is the repo root -- run.py reads the
generated MJCF from the relative path ``models/*.xml``, so the integration
tests must run from here regardless of where pytest was invoked.
"""

from __future__ import annotations

import os

import pytest

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))


@pytest.fixture
def repo_root() -> str:
    return REPO_ROOT


@pytest.fixture
def in_repo_root(monkeypatch):
    """Chdir to the repo root for the duration of a test."""
    monkeypatch.chdir(REPO_ROOT)
    return REPO_ROOT
