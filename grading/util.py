import re
import json

CAMEL_PATTERN = re.compile(r'(?<!^)(?=[A-Z])')
NON_AZ_PATTERN = re.compile(r'[^\w-]')

def insert_spaces_to_camel(CamelString:str):
    return CAMEL_PATTERN.sub(' ', CamelString)

def camel_to_spaced_snake(CamelString:str):
    return insert_spaces_to_camel(CamelString).lower()

def sortable_name_to_file_name(name:str):
    return NON_AZ_PATTERN.sub('', name).lower()

def find_next(segmented_path:list, locator:str):
    index = segmented_path.index(locator)
    return segmented_path[index+1]

def load_grading_jsons(json_ps, select_student=None):
    fname2json = {}
    for p in json_ps:
        fname = p.parent.parent.stem
        if select_student and fname != select_student:
            print(f"skip {fname}")
            continue
        print(f'loading {p}')
        with open(p, 'r') as f:
            j = json.load(f)
        print(j)
        manual = p.parent.joinpath('manual.json')
        if manual.exists():
            '''
            Patch "j" for manual updates
            '''
            print(f'loading {manual}')
            with open(manual, 'r') as f:
                mj = json.load(f)
            j['Points'].update(mj['Points'])
            j['TotalPoints'].update(mj['TotalPoints'])
            if 'Comment' in j:
                j['Comment'].update(mj['Comment'])
            else:
                j['Comment'] = mj['Comment']
        print(j)
        overall = 0.0
        for k,v in j['Points'].items():
            print(f"{k}: {v}")
            overall += v
        j['OVERALL'] = overall
        fname2json[fname] = j
        pts = j['OVERALL']
        # print(f'{fname}: {fname2stu[fname].id} Points {pts}')
        print(f'{fname}: Points {pts}')
    return fname2json


'''
Class Name (kname) to Rubric Title
'''
def build_rub_kname2title(rub_list):
    ret = {}
    for e in rub_list:
        ret[e[2]] = e[0]
    return ret
