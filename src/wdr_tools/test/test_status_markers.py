from wdr_tools.status_markers import StatusTracker, describe, heading_word


def test_tracker_follows_an_order_through_its_life():
    t = StatusTracker()
    t.on_event('ORD-1', 'queued', 'QUEUED', 'shelf_a1', 'pack_1')
    t.on_event('ORD-2', 'queued', 'QUEUED', 'shelf_b2', 'shipping')
    assert t.queue == ['ORD-1', 'ORD-2']
    t.on_event('ORD-1', 'started', '')
    t.on_event('ORD-1', 'state', 'TO_PICKUP')
    assert 'driving to pick-up shelf_a1' in t.robot_text()
    assert t.queue == ['ORD-2']
    t.on_event('ORD-1', 'succeeded', 'DONE')
    t.on_event('ORD-2', 'canceled', 'QUEUED')
    assert (t.delivered, t.canceled, t.queue, t.current) == (1, 1, [], None)
    t.on_event('', 'state', 'RETURNING_TO_DOCK')
    assert t.robot_text() == 'idle: returning to the charger'
    assert 'delivered 1' in t.board_text() and 'queue (0): empty' in t.board_text()


def test_descriptions_and_headings():
    assert describe('WAITING_FOR_UNLOAD', dropoff='pack_2') == 'waiting to be unloaded at pack_2'
    assert describe('SOMETHING_NEW') == 'something new'
    assert heading_word(0.0, 1.0) == 'north'
    assert heading_word(-1.0, 0.1) == 'west'
    assert heading_word(0.001, 0.0) == 'standing'
