from webxr_teleop.hardware.keyboard import TerminalKeys


def test_fragmented_arrows_do_not_trigger_escape_or_letter_commands():
    keys=TerminalKeys()
    assert keys.feed(b'\x1b',0)==[]
    assert keys.feed(b'[',.01)==[]
    assert keys.feed(b'C',.02)==[b'right']
    assert keys.feed(b'\x1b[D\x1bOA\x1b[B ',.03)==[b'left',b'up',b'down',b' ']


def test_escape_and_shutdown():
    keys=TerminalKeys()
    assert keys.feed(b'\x1b',1)==[]
    assert keys.feed(b'',1.06)==[b'escape']
    assert keys.feed(b'\x03',2)==[b'\x03']


def test_unknown_sequence_is_not_a_restart_or_sleep_command():
    keys=TerminalKeys()
    assert keys.feed(b'\x1b[1;5C',0)==[]
    assert keys.feed(b'',.1)==[]
