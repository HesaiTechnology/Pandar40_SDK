cp include/hesaiLidarSDK.h include/point_types.h sdk/pandoraSDK/
mkdir -p sdk/pandoraSDK/lib/`lsb_release -rs`
cp build/libhesaiLidarSDK.so sdk/pandoraSDK/lib/`lsb_release -rs`/
