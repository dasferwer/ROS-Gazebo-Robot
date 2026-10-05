import importlib.util
from pathlib import Path
import pytest
spec=importlib.util.spec_from_file_location('verification_rules',Path(__file__).resolve().parents[3]/'scripts/verification_rules.py')
rules=importlib.util.module_from_spec(spec)
spec.loader.exec_module(rules)


def graph():
    return {'tf_edge_publisher_gids':{edge:['gid1'] for edge in rules.EXPECTED_TF},
            'endpoints':{'/clock':{'publishers':[{'node':'simulation_bridge'}]}}}


def test_accepts_complete_tf_evidence():
    assert rules.tf_evidence_valid(graph())


@pytest.mark.parametrize('fault',['empty','missing','duplicate_parent','duplicate_publisher','clock'])
def test_rejects_incomplete_or_conflicting_evidence(fault):
    g=graph()
    if fault=='empty': g['tf_edge_publisher_gids']={}
    if fault=='missing': g['tf_edge_publisher_gids'].pop('map -> odom')
    if fault=='duplicate_parent': g['tf_edge_publisher_gids']['wrong -> base_link']=['gid2']
    if fault=='duplicate_publisher': g['tf_edge_publisher_gids']['map -> odom'].append('gid2')
    if fault=='clock': g['endpoints']['/clock']['publishers'].append({'node':'another_clock'})
    assert not rules.tf_evidence_valid(g)
