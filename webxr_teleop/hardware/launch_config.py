"""JSON launch defaults, validated by the same parser as CLI options."""
import argparse
import json
import math
from pathlib import Path


PATH_OPTIONS={'profile','output','lerobot_python','socket'}


def configured_argv(parser,argv):
    probe=argparse.ArgumentParser(add_help=False)
    probe.add_argument('--config',type=Path)
    config,_=probe.parse_known_args(argv)
    if config.config is None:return argv
    path=config.config.resolve()
    try:
        values=json.loads(path.read_text())
    except (OSError,ValueError) as exc:
        parser.error(f'Cannot read launch config {path}: {exc}')
    if not isinstance(values,dict):parser.error('Launch config must be a JSON object')
    actions={a.dest:a for a in parser._actions if a.option_strings and a.dest not in ('help','config')}
    defaults=[]
    for key,value in values.items():
        if key not in actions:parser.error(f'Unknown launch config setting: {key}')
        action=actions[key]
        option=action.option_strings[0]
        boolean=isinstance(action,(argparse._StoreTrueAction,argparse.BooleanOptionalAction))
        if boolean:
            if not isinstance(value,bool):parser.error(f'{key} must be a JSON boolean')
            if value:defaults.append(option)
            elif isinstance(action,argparse.BooleanOptionalAction):defaults.append('--no-'+option[2:])
            continue
        expected=action.type or type(action.default)
        if expected is Path or expected is str:
            valid=isinstance(value,str)
        elif expected is int:
            valid=isinstance(value,int) and not isinstance(value,bool)
        elif expected is float:
            valid=isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
        else:valid=False
        if not valid:parser.error(f'Invalid JSON value for {key}: {value!r}')
        if key in PATH_OPTIONS and not Path(value).is_absolute():value=str(path.parent/value)
        defaults.extend([option,str(value)])
    # Explicit CLI arguments occur last and override JSON defaults.
    return defaults+list(argv)
