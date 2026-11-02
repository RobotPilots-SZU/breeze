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
    llvm = pkgs.llvmPackages_latest;
    clang_tools_wrapped = pkgs.symlinkJoin {
      name = "clang-tools-wrapped";
      paths = [ llvm.clang-tools ];

      nativeBuildInputs = [ pkgs.makeWrapper ];

      postBuild = ''
        rm "$out/bin/clangd"

        makeWrapper \
          "${llvm.clang-tools}/bin/clangd" \
          "$out/bin/clangd" \
          --set LANG C \
          --set LC_ALL C
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
      ]) ++ [
        clang_tools_wrapped
      ];
    };
    shell-native =
      let
        env = {
          CLANGD_FLAGS="--query-driver=${pkgs.gcc_multi}/bin/g++";
          NIX_LDFLAGS = " -L${pkgs.pkgsi686Linux.gcc.cc.lib}/lib -L${pkgs.pkgsi686Linux.gcc.cc.libgcc}/lib";
        };
      in pkgs.mkShell ({
        inputsFrom = [ shell-base ];
        nativeBuildInputs = [
          pkgs.pkgsi686Linux.glibc
          pkgs.gcc_multi
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
