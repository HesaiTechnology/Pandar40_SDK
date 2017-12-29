cp include/pandoraSDK.h include/point_types.h sdk/pandoraSDK/
mkdir -p sdk/pandoraSDK/lib/`lsb_release -rs`
cp build/libpandoraSDK.so sdk/pandoraSDK/lib/`lsb_release -rs`/