import sys
from pathlib import Path


class FollowScenarioLauncher:
    def run(self) -> int:
        project_root = Path(__file__).resolve().parent
        source_root = project_root / "src"
        source_root_text = str(source_root)
        if source_root_text not in sys.path:
            sys.path.insert(0, source_root_text)

        from nd_sociomind.experiment.autonomous_navigation.follow_scenario.application.follow_scenario_application import FollowScenarioApplication

        return FollowScenarioApplication().run()


if __name__ == "__main__":
    raise SystemExit(FollowScenarioLauncher().run())
