{ ... }:
{
  perSystem = { inputs', pkgs, ... }:
  let
    inherit (inputs') breeze;
  in {
    devShells.default = pkgs.mkShell {
      name = "breeze-modules-dev-shell";
      inputsFrom = [ breeze.devShells.development ];
      packages = (with pkgs; [
          # 在这里按需添加其他包

      ]);
    };
  };
}
