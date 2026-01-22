import tomllib


def test_robotpy_dependencies() -> None:
    with open("pyproject.toml", "rb") as f:
        data = tomllib.load(f)

    project_dependencies = data["project"]["dependencies"]
    robotpy_table = data["tool"]["robotpy"]

    robotpy_components = ",".join(robotpy_table["components"])
    robotpy_version = robotpy_table["robotpy_version"]
    expected_robotpy_dependency = f"robotpy[{robotpy_components}]=={robotpy_version}"
    assert expected_robotpy_dependency in project_dependencies

    for dependency in robotpy_table["requires"]:
        assert dependency in project_dependencies
