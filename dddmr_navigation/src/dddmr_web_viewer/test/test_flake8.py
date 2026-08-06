from ament_flake8.main import main
import pytest


@pytest.mark.linter
def test_flake8():
    assert main(argv=[]) == 0
