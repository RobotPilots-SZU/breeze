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
    gcc_multi_wrapped = pkgs.symlinkJoin {
      name = "gcc-multi-locale-c";
      paths = [ pkgs.gcc_multi ];

      nativeBuildInputs = [ pkgs.makeWrapper ];

      postBuild = ''
        for exe in "$out"/bin/*; do
          if [ -f "$exe" ] && [ -x "$exe" ]; then
            wrapProgram "$exe" \
              --set LANG C \
              --set LC_ALL C
          fi
        done
      '';
    };
    shell-base = pkgs.mkShell {
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
      ]) ++ (with pkgs.llvmPackages_latest; [
        clang-tools
      ]);
    };
    shell-native =
      let
        env = {
          CLANGD_FLAGS="--query-driver=${gcc_multi_wrapped}/bin/g++";
          NIX_LDFLAGS = " -L${pkgs.pkgsi686Linux.gcc.cc.lib}/lib -L${pkgs.pkgsi686Linux.gcc.cc.libgcc}/lib";
        };
      in pkgs.mkShell ({
        inputsFrom = [ shell-base ];
        nativeBuildInputs = [
          pkgs.pkgsi686Linux.glibc
          gcc_multi_wrapped
        ];
        passthru.env = env;
      } // env);
    shell-arm =
      let
        env = {
          CLANGD_FLAGS="--query-driver=${sdk}/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-g++";
        };
      in pkgs.mkShell ({
        inputsFrom = [ shell-base ];
        passthru.env = env;
      } // env);
    in {
    devShells = {
      base = shell-base;
      arm = shell-arm;
      native = shell-native;
      default = shell-arm;
    };
  };
}
