"""Read-only wall-based scale measurements; no map accuracy acceptance claim."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from PIL import Image, ImageDraw


def cell_centres(message):
    info = message['info']
    width, height = info['width'], info['height']
    resolution = info['resolution']
    if width <= 0 or height <= 0 or not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('Invalid map geometry')
    cells = np.asarray(message['data']).reshape(height, width)
    q = info['origin']['orientation']
    if abs(q['x']) > 1e-6 or abs(q['y']) > 1e-6:
        raise ValueError('Only planar map origins are supported')
    yaw = 2*np.arctan2(q['z'], q['w'])
    rows, columns = np.indices(cells.shape)
    x = (columns+.5)*resolution
    y = (rows+.5)*resolution
    p = info['origin']['position']
    world_x = p['x'] + np.cos(yaw)*x - np.sin(yaw)*y
    world_y = p['y'] + np.sin(yaw)*x + np.cos(yaw)*y
    return cells, world_x, world_y


def fit_wall(points, axis, bin_size=.5):
    """One median coordinate per longitudinal bin, then fit a straight line."""
    if len(points) < 10:
        raise ValueError('Insufficient wall cells')
    normal = points[:, axis]
    along = points[:, 1-axis]
    bins = np.floor(along/bin_size).astype(int)
    centres = np.array([[np.median(along[bins==i]), np.median(normal[bins==i])]
                        for i in np.unique(bins)])
    if len(centres) < 6 or np.ptp(centres[:, 0]) < 3:
        raise ValueError('Insufficient longitudinal wall support')
    slope, intercept = np.polyfit(centres[:, 0], centres[:, 1], 1)
    residual = (normal - slope*along - intercept)/np.sqrt(1+slope*slope)
    return {'cell_count': len(points), 'occupied_bins': len(centres),
            'span_m': float(np.ptp(along)), 'slope': float(slope),
            'intercept_m': float(intercept),
            'median_abs_residual_m': float(np.median(np.abs(residual))),
            'p95_abs_residual_m': float(np.percentile(np.abs(residual),95)),
            'fit_description': 'normal_coordinate = slope * along_coordinate + intercept; medians in 0.5 m bins'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--world',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    with args.map.open() as stream:
        m=next(d for d in yaml.safe_load_all(stream) if isinstance(d,dict) and 'data' in d)
    includes=ET.parse(args.world).getroot().find('world').findall('include')
    poses={i.findtext('name'): [float(v) for v in i.findtext('pose').split()]
           for i in includes if i.findtext('pose')}
    right=poses['forest_wall1'][0];left=poses['forest_wall4'][0];top=poses['forest_wall10'][1]
    if not (np.isclose(right,12.5) and np.isclose(left,-12.5) and np.isclose(top,12.5)):
        raise ValueError('This ROI configuration requires the inspected large_demo wall layout')
    cells,x,y=cell_centres(m)
    structures=cells==100
    # Explicit candidate ROIs: frame alignment is an assumption to inspect, not a fitted correction.
    rois={
        'left': structures & (x>=left-.5)&(x<left+.5)&(y>=-2)&(y<8),
        'right': structures & (x>=right-.5)&(x<right+.5)&(y>=-2)&(y<8),
        'top': structures & (y>=top-.5)&(y<top+.5)&(x>=-10)&(x<10),
    }
    fits={name:fit_wall(np.column_stack([x[mask],y[mask]]),1 if name=='top' else 0)
          for name,mask in rois.items()}
    nominal=right-left
    widths={str(level):float((fits['right']['slope']-fits['left']['slope'])*level
                            +fits['right']['intercept_m']-fits['left']['intercept_m'])
            for level in [0.,3.,6.]}
    measured=widths['3.0']
    report={'status':'MEASURED_NOT_ACCEPTANCE', 'map':str(args.map.resolve()),
            'map_sha256':hashlib.sha256(args.map.read_bytes()).hexdigest(),
            'world':str(args.world.resolve()),
            'world_sha256':hashlib.sha256(args.world.read_bytes()).hexdigest(),
            'frame_id':m['header']['frame_id'],'resolution_m':m['info']['resolution'],
            'nominal_wall_plane_spacing_m':nominal,'measured_x_separation_at_y_m':widths,
            'scale_ratio_at_y3':measured/nominal,'signed_width_error_m':measured-nominal,
            'absolute_relative_width_error_percent':abs(measured/nominal-1)*100,
            'fits':fits,
            'roi_definition':'Left/right: x within 0.5 m of nominal wall, y [-2,8). Top: y within 0.5 m of nominal wall, x [-10,10).',
            'limitations':[
                'Association assumes map axes approximately align with world axes; verify overlay. No coordinate alignment was estimated.',
                'SDF visual wall planes are the nominal reference; 0.01 m collision thickness and actual sensor rendering can change observed surfaces.',
                'Cell centres quantize measurements at 0.1 m resolution. Sub-cell fit values do not establish sub-cell absolute accuracy.',
                'Width is x separation at common y, not a general perpendicular distance between nonparallel lines.',
                'Residual describes wall scatter/thickness, not independent global map accuracy.',
                'No team-agreed acceptance tolerance or confirmed tree landmarks supplied.']}
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'scale_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    rgb=np.full((*cells.shape,3),205,dtype=np.uint8)
    rgb[cells==0]=254;rgb[structures]=0
    for name,color in [('left',(220,50,50)),('right',(50,90,230)),('top',(20,160,60))]:
        rgb[rois[name]]=color
    im=Image.fromarray(np.flipud(rgb)).resize((800,800),Image.NEAREST)
    canvas=Image.new('RGB',(800,850),'white');canvas.paste(im,(0,0))
    draw=ImageDraw.Draw(canvas)
    draw.text((10,810),'Red: left wall | Blue: right wall | Green: top wall',fill='black')
    draw.text((10,828),'Candidate association; measurements are not an accuracy acceptance result.',fill='black')
    canvas.save(args.output/'wall_regions.png')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
