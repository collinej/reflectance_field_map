project(
    'reflectance_field_map',
    'cpp',
    default_options : [
        'cpp_std=c++20',
        'werror=true',
        'warning_level=3',
        'buildtype=debugoptimized'
    ]
)

external_inc = include_directories('cereal/include')

subdir('rfm')
subdir('examples')
