add_executable( direct_sparse_mono direct_sparse_mono.cpp )
target_link_libraries( direct_sparse_mono
    ds_vo
    ${THIRD_PARTY_LIBS}
)
