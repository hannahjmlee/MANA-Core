zip_dir="benchmarks/maps/zip_files"
dest_dir="benchmarks/maps"

for zip_file in "$zip_dir"/*.zip; do
  # Unzip each file into the destination directory
  unzip -o "$zip_file" -d "$dest_dir"
done


zip_dir="benchmarks/scenarios/zip_files"
dest_dir="benchmarks/scenarios"

for zip_file in "$zip_dir"/*.zip; do
  # Unzip each file into the destination directory
  unzip -o "$zip_file" -d "$dest_dir"
done

mv benchmarks/scenarios/scen-random/* benchmarks/scenarios
rmdir benchmarks/scenarios/scen-random
