add_executable( direct_sparse_mono direct_sparse_mono.cpp )
target_link_libraries( direct_sparse_mono
    ds_vo
    ${THIRD_PARTY_LIBS}
)

add_executable( basic_direct_vo basic_direct_vo.cpp )
target_link_libraries( basic_direct_vo
        ds_vo
        ${THIRD_PARTY_LIBS}
)

add_executable( test_sparse_align_pair test_sparse_align_pair.cpp )
target_link_libraries( test_sparse_align_pair
        ds_vo
        ${THIRD_PARTY_LIBS}
)