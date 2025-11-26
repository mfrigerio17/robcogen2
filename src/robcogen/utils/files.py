import pathlib

def open_utf8_reading(filepath):
    return open(filepath, mode="r", encoding="utf-8")

def base_path(file):
    return pathlib.Path(file).resolve().parent

def compose_paths(directories):
    root = pathlib.Path()
    for folder in directories:
        root = root / folder
    return root

class FileWriter:
    '''
    A simple wrapper of a base path, that creates files at such location
    '''

    def __init__(self, base_path):
        p = pathlib.Path(base_path)
        if not p.exists():
            p.mkdir(parents=True)
        self.path = p

    def subDir(self, subdir):
        return FileWriter(self.path / subdir)

    def genFile(self, file_name, text_content):
        fpath = self.path / file_name
        f = open(fpath, mode="w", encoding="utf-8")
        f.write(text_content)
        f.close()

    @property
    def basePath(self):
        return self.path
