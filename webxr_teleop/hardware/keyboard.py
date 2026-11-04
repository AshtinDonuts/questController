"""Incremental terminal key decoding; escape sequences may span reads."""


class TerminalKeys:
    def __init__(self):
        self.pending=b''
        self.since=0.

    def feed(self, data, now):
        if data:
            if not self.pending:self.since=now
            self.pending+=data
        result=[]
        while self.pending:
            if self.pending[0]!=27:
                result.append(self.pending[:1]);self.pending=self.pending[1:]
                continue
            if len(self.pending)>=3 and self.pending[1:2] in (b'[',b'O'):
                key={65:b'up',66:b'down',67:b'right',68:b'left'}.get(self.pending[2])
                if key:
                    result.append(key);self.pending=self.pending[3:];continue
            if len(self.pending)==1 and now-self.since>=.05:
                result.append(b'escape');self.pending=b'';continue
            if now-self.since>=.05:
                # Unknown/incomplete control sequences never become letter commands.
                self.pending=b''
            break
        if not self.pending:self.since=now
        return result
