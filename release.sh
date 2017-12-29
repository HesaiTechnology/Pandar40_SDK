cp include/pandoraSDK.h include/point_types.h release/pandoraSDK/
mkdir -p release/pandoraSDK/lib/`lsb_release -rs`
cp build/libpandoraSDK.so release/pandoraSDK/lib/`lsb_release -rs`/