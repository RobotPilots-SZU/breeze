{ ... }:
{
  perSystem = { pkgs, inputs', ... }: 
  let
    zephyrPackages = inputs'.zephyr-nix.packages;
  in 
  {
    devShells =
    let
      workspace = pkgs.mkShell {
        name = "breeze-dev-workspace";

        packages = with pkgs; [
          cmake
          ninja
          gperf
          ccache
          dfu-util
          dtc
          wget
          file
          gnumake
          gcc
          gdb
          openocd
          git
          python3
          SDL2
        ] ++ ( with zephyrPackages; [
          (sdk.override {
            targets = [
              "arm-zephyr-eabi"
            ];
          })
          (pythonEnv.override {
            packageOverrides = _self: super: {
              tree-sitter-cmake = super.tree-sitter-grammars.tree-sitter-cmake;
            };
          })
          hosttools
          clang-tools
        ]);
      };
    in {
      inherit workspace;
      default = workspace;
    };
  };
}
