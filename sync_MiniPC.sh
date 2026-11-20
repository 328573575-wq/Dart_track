set -e
exec rsync -avzP \
  --exclude='build/' --exclude='install/' --exclude='log/' \
  --exclude='runs/' --exclude='MvSdkLog/' \
  /home/linzikang/dart_ws/ pc99@192.168.2.199:/home/pc99/workspace/Dart_track/