import re

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

'''
Class Name (kname) to Rubric Title
'''
def build_rub_kname2title(rub_list):
    ret = {}
    for e in rub_list:
        ret[e[2]] = e[0]
    return ret
