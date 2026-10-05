#!/usr/bin/env python3
"""Проверяет сохранённый результат живого запуска, а не наличие файлов проекта."""
import json
from pathlib import Path
import sys
from verification_rules import tf_evidence_valid

path=Path(sys.argv[1])
a=json.loads((path/'audit.json').read_text())
route=json.loads((path/'route.json').read_text())
checks={
    'route_succeeded':route['success'],
    'gps_lidar_depth_received':all(a['messages'].get(t,0)>20 for t in ['/gps/fix','/scan','/depth/points']),
    'clock_monotonic':a['messages'].get('/clock',0)>100 and a['clock_regressions']==0,
    'tf_connected':all(a['map_transforms'].get(t) is not None for t in ['base_link','lidar_link','camera_optical_frame','gps_link']),
    'depth_obstacle_consumed':a['depth_low_obstacle_marked_updates']>0,
    'contact_sensors_received':all(a['messages'].get('/contacts/'+t,0)>100 for t in ['body','left','right']),
    'no_obstacle_contacts':not a['obstacle_contacts'],
    'no_geometry_overlap':a['geometry_overlap_samples']==0 and a['minimum_sat_separation_m']>0,
    'ground_truth_goal_error':a['ground_truth_finish_error_m']<=.25,
}
if (path/'graph.json').exists():
    graph=json.loads((path/'graph.json').read_text())
    for topic,label in [('/scan','lidar'),('/depth/points','depth')]:
        names={e['node'] for e in graph['endpoints'][topic]['subscribers']}
        checks[label+'_costmap_consumers']={'local_costmap','global_costmap'}<=names
    checks['one_tf_publisher_per_edge']=tf_evidence_valid(graph)
else:
    checks['graph_evidence_exists']=False
(path/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
print(json.dumps(checks,ensure_ascii=False))
sys.exit(0 if all(checks.values()) else 1)
