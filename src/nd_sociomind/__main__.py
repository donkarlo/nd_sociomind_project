from typing import Optional, List

from nd_sociomind.cli import Cli

class Main:
    def __init__(self, faked_cli_args: Optional[List[str]] = None):
        self._sociomind_cli = Cli(faked_cli_args)
    def run(self):
        self._sociomind_cli.run()


def main(faked_cli:Optional[List[str]]=None)->None:
    main_obj:Main = Main(faked_cli)
    main_obj.run()

if __name__== "__main__":
    """
    From private line in the phd-venv run nd_world_project xpr oldest learning
    """
    faked_cli_prog_name = ["nd_world_project"]
    faked_cli_args = ["experience" , "normal" , "remember"]


    faked_cli = faked_cli_prog_name + faked_cli_args

    main(faked_cli)