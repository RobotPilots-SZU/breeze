{ ... }:
{
  perSystem = { inputs', pkgs, ... }:
  let
    inherit (inputs') breeze;
  in {
    devShells.default = pkgs.mkShell ({
      name = "breeze-modules-dev-shell";
      inputsFrom = [ breeze.devShells.arm ];
      packages = (with pkgs; [
          # 在这里按需添加其他包

      ]);
    } // breeze.devShells.arm.env);
  };
}
