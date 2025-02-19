for dir in */; do
  cd $dir
  cargo update
  cd ..
done
