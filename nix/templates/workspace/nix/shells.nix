{ ... }:
{
  perSystem = { inputs', pkgs, ... }:
  let
    inherit (inputs') breeze;
  in {
    devShells.default = pkgs.mkShell {
      name = "breeze-modules-dev-shell";
      inputsFrom = [ breeze.devShells.workspace ];
    };
  };
}

