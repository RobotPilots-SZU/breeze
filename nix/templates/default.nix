{ ... }:
{
  flake.templates.workspace = {
    path = ./workspace;
    description = "A Nix-enabled Breeze West workspace";
    welcomeText = ''
      Breeze workspace initialized.
      Run `nix develop` to enter the workspace development environment.
    '';
  };
}
