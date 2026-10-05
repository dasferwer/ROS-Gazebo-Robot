EXPECTED_TF = {
    'map -> odom', 'odom -> base_footprint', 'base_footprint -> base_link',
    'base_link -> lidar_link', 'base_link -> gps_link', 'base_link -> camera_link',
    'camera_link -> camera_optical_frame', 'base_link -> left_wheel',
    'base_link -> right_wheel', 'base_link -> front_caster', 'base_link -> rear_caster',
}


def tf_evidence_valid(graph):
    edges=graph.get('tf_edge_publisher_gids',{})
    if not EXPECTED_TF <= set(edges) or any(len(gids)!=1 for gids in edges.values()):
        return False
    parents={}
    for edge in edges:
        parent,child=edge.split(' -> ')
        if child in parents and parents[child]!=parent:
            return False
        parents[child]=parent
    clock=graph.get('endpoints',{}).get('/clock',{}).get('publishers',[])
    return len(clock)==1
