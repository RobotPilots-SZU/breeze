{ ... }:
{
  perSystem = { pkgs, inputs', ... }:
  let
    zephyrPackages = inputs'.zephyr-nix.packages;
    sdk = zephyrPackages.sdk.override {
      targets = [
        "arm-zephyr-eabi"
      ];
    };
    breeze-shell = pkgs.mkShell {
      name = "breeze-shell";
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
        gdb
        openocd
        git
        python3
        SDL2
      ] ++ (with zephyrPackages; [
        sdk
        (pythonEnv.override {
          packageOverrides = _self: super: {
            tree-sitter-cmake = super.tree-sitter-grammars.tree-sitter-cmake;
          };
        })
        hosttools
        clang-tools
      ]);

      shellHook = ''
        export CLANGD_FLAGS="--query-driver=${sdk}/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-g++"
      '';
    };
  in {
    devShells = {
      development = breeze-shell;
      default = breeze-shell;
    };
  };
}
