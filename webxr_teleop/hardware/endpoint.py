"""Own the local control endpoint and recover leftovers from interrupted runs."""
import errno
import fcntl
import os
import re
from pathlib import Path
import socket
import stat


def is_hardware_process(args):
    """Match the interpreter's module, not text buried in conda/shell arguments."""
    if not args:return False
    args=[os.fsdecode(a) for a in args if a]
    if not args or not re.fullmatch(r'python(?:[0-9]+(?:\.[0-9]+)*)?',Path(args[0]).name):return False
    i=1
    while i<len(args):
        arg=args[i]
        if arg=='-m':break
        if arg in ('-W','-X'):
            i+=2;continue
        if arg.startswith(('-W','-X')) or arg in ('-u','-B','-E','-s','-S','-I','-O','-OO','-q'):
            i+=1;continue
        return False  # Script path or -c: later -m tokens are not interpreter options.
    if i+1>=len(args) or args[i+1] not in ('webxr_teleop.hardware.run','webxr_teleop.hardware.worker'):return False
    options=args[i+2:]
    if '--mode=mock' in options or any(a=='--mode' and b=='mock' for a,b in zip(options,options[1:])):return False
    if options and options[0] in ('status','stop','reset','sleep'):return False
    return True


def hardware_processes():
    result=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name)==os.getpid():continue
        try:
            if entry.stat().st_uid!=os.getuid():continue
            args=(entry/'cmdline').read_bytes().split(b'\0')
            if is_hardware_process(args):result.append(int(entry.name))
        except (OSError,ProcessLookupError):pass
    return result


def bind_control(path):
    path=Path(path)
    directory=path.parent;directory.mkdir(mode=0o700,parents=True,exist_ok=True)
    if directory.stat().st_uid!=os.getuid() or directory.stat().st_mode&0o077:
        raise ValueError('Control socket directory must be private (0700)')
    lock=open(str(path)+'.lock','a')
    server=None
    try:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another supervisor owns this control endpoint; stop that session before relaunching') from None
        try:before=path.lstat()
        except FileNotFoundError:before=None
        if before is not None:
            if not stat.S_ISSOCK(before.st_mode) or before.st_uid!=os.getuid():
                raise RuntimeError('Refusing to replace a non-socket or unowned control endpoint')
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as probe:
                probe.settimeout(.5)
                try:probe.connect(str(path))
                except OSError as exc:
                    if exc.errno!=errno.ECONNREFUSED:raise
                else:
                    raise RuntimeError('A live supervisor is listening; stop that session before relaunching')
            owners=hardware_processes()
            if owners:raise RuntimeError(f'Hardware processes still running (PIDs {owners}); resolve the old session before relaunching')
            after=path.lstat()
            if (before.st_dev,before.st_ino)!=(after.st_dev,after.st_ino):
                raise RuntimeError('Control endpoint changed during inspection; retry')
            path.unlink()
            print(f'Removed stale control socket: {path}',flush=True)
        server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        server.bind(str(path));server.listen(4);server.setblocking(False)
        return server,lock
    except BaseException:
        if server:server.close()
        lock.close()
        raise
