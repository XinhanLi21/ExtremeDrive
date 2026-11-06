"""Annotation-conditional solid-line violation rates from executed trajectories."""
def violation_rates(rows, kind='white', window_ids=None):
    if kind not in ('white','yellow'):
        raise ValueError('only white/yellow solid markings are supported')
    selected=[r for r in rows if window_ids is None or r['window_id'] in window_ids]
    available=[r for r in selected if r.get('details',{}).get(kind,{}).get('available_marking',False)]
    events=[e for r in available for e in r.get('events',[]) if e['type']==kind+'_solid']
    violated=[r['window_id'] for r in available if any(e['type']==kind+'_solid' for e in r.get('events',[]))]
    return dict(total_windows=len(selected),annotated_windows=len(available),unavailable_windows=len(selected)-len(available),
                violation_windows=len(violated),violation_rate=len(violated)/len(available) if available else None,
                event_count=len(events),violating_window_ids=violated,
                denominator='windows with a locally available annotated solid marking; conditional on annotations',
                event_count_scope='sum within evaluation windows; not deduplicated across overlapping windows')
