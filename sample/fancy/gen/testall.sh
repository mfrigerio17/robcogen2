if [ "$#" -ne 4 ]; then
  echo "Usage: $0 <PATH to RobCoGen root> <PATH to spatial_v2 root> <PATH to generated C++> <PATH to generated Octave>" >&2
  exit 1
fi
if ! [ -d "$1" ]; then
  echo "$1 not a directory" >&2
  exit 1
fi
if ! [ -d "$2" ]; then
  echo "$2 not a directory" >&2
  exit 1
fi
if ! [ -d "$3" ]; then
  echo "$3 not a directory" >&2
  exit 1
fi
if ! [ -d "$4" ]; then
  echo "$4 not a directory" >&2
  exit 1
fi

abspath_rcg_root=$(cd $1 && pwd -P)
abspath_spatial_v2=$(cd $2 && pwd -P)
abspath_gen_cpp=$(cd $3 && pwd -P)
abspath_gen_octave=$(cd $4 && pwd -P)

origin=$(cd -- "$(dirname -- "$0")" &> /dev/null && pwd)
dest=`mktemp --directory`

cd $dest && cmake "$abspath_gen_cpp" && make -j3
if [ $? != 0 ]
then
    echo "Failed to build the C++ code"
    exit 1
fi

cd $origin

echo "Running C++ consistency tests..."
$dest/test-cons

echo "Running tests for both C++ and Octave ..."
octave --quiet testall.m $abspath_spatial_v2 "$abspath_rcg_root/testing/src" $abspath_gen_octave $dest
