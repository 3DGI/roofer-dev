Output directory for tests.

Run all test cases.

```shell
$exe -c tests/config-all-ahn.toml --verbose
$exe -c tests/config-yoc-2019-completely-new.toml --verbose
$exe -c tests/config-yoc-2010-ahn.toml --verbose
$exe -c tests/config-yoc-2022-low-coverage.toml --verbose
$exe -c tests/config-sheds-garage.toml --verbose
$exe -c tests/config-bag-underground.toml --verbose
```

Merge the individual geopackages into one.

```shell
paths_n=$(find -L './all-ahn/objects/' -path './all-ahn/objects/*/*.gpkg')
paths="$(tr '\n' ' '  <<<$paths_n)"
ogrmerge.py -o ./all-ahn/objects.gpkg $paths -f GPKG -single

paths_n=$(find -L './bag-underground/objects/' -path './bag-underground/objects/*/*.gpkg')
paths="$(tr '\n' ' '  <<<$paths_n)"
ogrmerge.py -o ./bag-underground/objects.gpkg $paths -f GPKG -single

paths_n=$(find -L './sheds-garage/objects/' -path './sheds-garage/objects/*/*.gpkg')
paths="$(tr '\n' ' '  <<<$paths_n)"
ogrmerge.py -o ./sheds-garage/objects.gpkg $paths -f GPKG -single

paths_n=$(find -L './yoc-2010-ahn/objects/' -path './yoc-2010-ahn/objects/*/*.gpkg')
paths="$(tr '\n' ' '  <<<$paths_n)"
ogrmerge.py -o ./yoc-2010-ahn/objects.gpkg $paths -f GPKG -single

paths_n=$(find -L './yoc-2019-completely-new/objects/' -path './yoc-2019-completely-new/objects/*/*.gpkg')
paths="$(tr '\n' ' '  <<<$paths_n)"
ogrmerge.py -o ./yoc-2019-completely-new/objects.gpkg $paths -f GPKG -single

paths_n=$(find -L './yoc-2022-low-coverage/objects/' -path './yoc-2022-low-coverage/objects/*/*.gpkg')
paths="$(tr '\n' ' '  <<<$paths_n)"
ogrmerge.py -o ./yoc-2022-low-coverage/objects.gpkg $paths -f GPKG -single
```