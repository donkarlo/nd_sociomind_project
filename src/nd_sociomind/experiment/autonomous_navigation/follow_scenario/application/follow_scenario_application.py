import sys

from PySide6.QtWidgets import QApplication

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.data_source_settings import DataSourceSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.data_source_locator import DataSourceLocator
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.window.follow_scenario_window import FollowScenarioWindow


class FollowScenarioApplication:
    def run(self) -> int:
        qt_application = QApplication.instance()
        if qt_application is None:
            qt_application = QApplication(sys.argv)

        qt_application.setApplicationName("Sociomind Follow Scenario")
        qt_application.setOrganizationName("nd_sociomind")

        scenario_settings = ScenarioSettings()
        data_source_settings = DataSourceSettings()
        leader_path, follower_path = DataSourceLocator(data_source_settings).locate()
        window = FollowScenarioWindow(leader_path, follower_path, scenario_settings, data_source_settings)
        window.show()
        return qt_application.exec()
