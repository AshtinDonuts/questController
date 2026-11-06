"""Operator-facing recording progress, shared by Quest, Rerun and the terminal."""


def clock(seconds):
    seconds=max(0,int(seconds))
    return f'{seconds//60}:{seconds%60:02d}'


def recording_status(config, recorder, *, state, reason, now_ns, aligned=False,
                     prepared=False, returning=False, release_pending=False,
                     exiting=False, fault=False, result='', alignment_hand='right'):
    phase=recorder.get('phase','warming')
    saved=recorder.get('saved',0)
    target=config.get('num_episodes',0)
    error=recorder.get('error')
    complete=bool(target and saved>=target)
    if fault or error or state=='FAULT':
        stage,title,hint='failed','STOPPED / FAULT','Motion held. Resolve the fault before continuing.'
    elif state in ('PARKING','DISABLING'):
        stage,title,hint='sleeping','SLEEPING','Wait for sleep arrival and torque-off.'
    elif state=='PARKED':
        stage,title,hint='parked','SLEEP COMPLETE','Torque off. Session closing.' if exiting else 'Torque off. Start a new session to continue.'
    elif release_pending or state=='RELEASING':
        stage,title,hint='releasing','OPENING GRIPPERS','Arm joints held. Wait for full aperture and settling.'
    elif returning and state in ('RESTART_PREPARING','OPENING'):
        stage,title,hint='returning','RETURNING TO OPENING','Wait for arrival; then release grips and press X to align.'
    elif phase in ('pausing','paused','resuming'):
        stage,title,hint='paused','RECORDING PAUSED','Restore tracking, release grips and press X to align and resume this episode.'
    elif state=='READY' and not prepared:
        stage,title,hint='hold','OPENING INCOMPLETE','Release grips. X: align here, then → to record; ↑: optional return to opening.'
    elif state in ('DISARMED','OPENING','RESTART_PREPARING'):
        stage,title,hint='opening','OPENING','Wait for opening arrival and settling.'
    elif phase in ('warming','starting','saving'):
        stage=phase
        title={'warming':'CAMERA WARMUP','starting':'STARTING EPISODE','saving':'SAVING EPISODE'}[phase]
        hint='Wait for REC before moving.' if phase!='saving' else 'File finalizing. Arm joints held.'
    elif phase=='recording' and state=='ARMED':
        stage,title,hint='recording','RECORDING','A / →: save | B / ←: discard + return | Space: STOP'
    elif complete:
        stage,title,hint='complete','COLLECTION COMPLETE','Esc: sleep and exit | S / hold Y: sleep'
    elif aligned and state=='ARMED':
        stage,title,hint='ready','READY TO RECORD','Release grips and wait for settling. A / →: record from current pose.'
    elif prepared and state in ('READY','ARMED','HOLD'):
        stage,title,hint='align','ALIGN CONTROLLERS',f'Release active grips, point {alignment_hand} controller forward, press X.'
    elif phase=='finished' and not recorder.get('last_status')=='aborted':
        stage,title,hint='between','EPISODE '+recorder.get('last_status','finished').upper(),'A / ↑: return to opening | Esc: sleep and exit'
    elif state=='ARMED':
        stage,title,hint='align','ALIGN CONTROLLERS',f'Release active grips, point {alignment_hand} controller forward, press X.'
    else:
        stage,title,hint='hold','POWERED HOLD','Motion held. Follow the status message before resuming.'
    start=recorder.get('start_ns')
    end=recorder.get('end_ns',now_ns)
    boundary=min(now_ns,end,recorder.get('pause_ns',now_ns))
    elapsed=max(0,(boundary-start-recorder.get('paused_ns',0))/1e9) if start is not None else 0
    duration=config.get('episode_time_s',0)
    timing=f'{clock(elapsed)} / {clock(duration)}' if duration else f'{clock(elapsed)} (manual)'
    if duration and phase=='recording':timing+=f' · {clock(max(0,duration-elapsed))} left'
    count=f'{saved} / {target or "∞"} saved'
    attempt=recorder.get('episode',-1)+1
    return dict(phase=stage,title=title,hint=hint,task=config['task'],episode=attempt,
        saved=saved,target_episodes=target or '∞',elapsed_s=elapsed,
        episode_time_s=duration,remaining_s=max(0,duration-elapsed) if duration else None,
        episode_progress=min(1,elapsed/duration) if duration else None,
        progress_text=timing,summary=f'{title} | {timing} | {count} | {hint} | {error or reason}'+(f' | {result}' if result else ''),
        message=reason,recording_error=error,recording_result=result,
        recorder_phase=phase,last_status=recorder.get('last_status'),
        cameras=recorder.get('cameras',{}))


def status_markdown(status):
    cameras=', '.join(f'{name}: {"OK" if c.get("healthy") else "UNAVAILABLE"}'
                     for name,c in status.get('cameras',{}).items()) or 'No cameras configured'
    return (f'# {status["title"]}\n\n'
            f'## {status["progress_text"]}\n\n'
            f'**Saved: {status["saved"]} / {status["target_episodes"]}**'
            f' · Episode attempt: {status["episode"] or "—"}\n\n'
            f'**Next: {status["hint"]}**\n\n'
            f'Task: {status["task"]}\n\n'
            f'{status["message"]}\n\n'
            f'Capture: {status["recorder_phase"]} · {cameras}\n\n'
            f'{status.get("recording_error") or status.get("recording_result") or ""}')


def recording_toggle_command(phase, *, aligned, state, prepared):
    """A saves an active episode, starts aligned capture, or prepares a return."""
    if phase in ('recording','paused'):
        return 'record-save'
    if phase == 'finished' and not prepared and not (aligned and state == 'ARMED'):
        return 'record-next'
    return 'record-start'
