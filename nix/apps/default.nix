{ self, ... }:
{
  perSystem = { pkgs, inputs', ... }:
  let
    runtime = with pkgs; [
      git
      coreutils
      nix
      (inputs'.zephyr-nix.packages.pythonEnv.override {
        packageOverrides = _final: prev: {
          tree-sitter-cmake = prev.tree-sitter-grammars.tree-sitter-cmake;
        };
      })
    ];
    init = pkgs.writeShellApplication {
      name = "breeze-init";
      runtimeInputs = runtime;
      text = ''
        usage() {
          printf 'Usage: breeze-init [workspace-directory]\n' >&2
          exit 2
        }

        destination="''${1:-breeze-workspace}"
        [[ "$#" -le 1 ]] || usage

        if [[ -e "$destination" ]] && [[ -n "$(ls -A "$destination")" ]]; then
          printf 'Destination is not empty: %s\n' "$destination" >&2
          exit 1
        fi

        mkdir -p "$destination"
        (
          cd "$destination"
          west init -m https://github.com/RobotPilots-SZU/breeze.git .
          west update
          nix flake init -t "${self}#workspace"
        )
        printf '\nWorkspace ready at %s\n' "$destination"
        printf 'Enter it with: cd %q\n' "$destination"
        printf 'Start the Nix environment with: nix develop\n'
      '';
    };
  in
  {
    packages.init = init;
    apps.init = {
      type = "app";
      program = "${init}/bin/breeze-init";
      meta.description = "Initialize a Breeze West workspace";
    };
  };
}
