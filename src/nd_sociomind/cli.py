from nd_utility.ui.cli.cli import Cli as CliBase
from experiment.cli import Cli as SociomindExperimentCli

class Cli(CliBase):
    def run(self)-> None:
        if self._args[0] == "experiment":
            xpr_cli = SociomindExperimentCli(self.get_program_args())
            xpr_cli.enliven()