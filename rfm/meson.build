src = [
    'angle_range.cpp',
    'discretized_angle_grid.cpp',
    'reflectance_field_map.cpp',
    'rfm_builder.cpp',
    'rfm_dynamic_filter.cpp',
    'rfm_utils.cpp',
    'point_cloud_2d.cpp',
    'timestamp.cpp'
]

rfm_lib = static_library(
    'rfm',
    src,
    include_directories : [ external_inc ],
)

rfm_dep = declare_dependency(
    link_with : rfm_lib,
    include_directories : [ external_inc, '../' ]
)
