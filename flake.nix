{
  description = "Basic rpi pico development shell";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/master";
  inputs.flake-utils.url = "github:numtide/flake-utils";

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
    let
      pico-sdk151 = with pkgs; (pico-sdk.overrideAttrs (o:
        rec {
        pname = "pico-sdk";
        version = "1.5.1";
        src = fetchFromGitHub {
          fetchSubmodules = true;
          owner = "raspberrypi";
          repo = pname;
          rev = version;
          sha256 = "sha256-GY5jjJzaENL3ftuU5KpEZAmEZgyFRtLwGVg3W1e/4Ho=";
        };
        }));
      pkgs = nixpkgs.legacyPackages.${system};
    in {
        devShell = pkgs.mkShell {
          nativeBuildInputs = with pkgs; [
            pico-sdk151
            cmake
            clang-tools
            gcc-arm-embedded
            ];
          shellHook = ''
            export PICO_SDK_PATH="${pico-sdk151}/lib/pico-sdk"
            '';
          };
      });
}