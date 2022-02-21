#!/usr/bin/env python3
import argparse
from random import random

'''
this file reads the version of the project from __init__ and push everything with tag
when tag is added, Travis.ci will upload the project to pip
'''

import os
import sys
import codecs
from git import Repo
import argparse
import subprocess


class Updater:
    def __init__(self, package_name) -> None:

        self.package = package_name
        self.path_to_git_repo = os.path.abspath(os.path.dirname(__file__))  # make sure .git folder is properly configured
        self.version = self.get_version(self.package + "/__init__.py")
        print(f"version of package '{self.package}': {self.version}")

        self.commit_message = f'version bump to {self.version}'
        self.commit_tag = f'v{self.version}'

    def read(self, rel_path):
        here = os.path.abspath(os.path.dirname(__file__))
        with codecs.open(os.path.join(here, rel_path), 'r') as fp:
            return fp.read()

    def get_version(self, rel_path):
        for line in self.read(rel_path).splitlines():
            if line.startswith('__version__'):
                delim = '"' if '"' in line else "'"
                return line.split(delim)[1]
        else:
            raise RuntimeError("Unable to find version string.")


    def git_update(self, push=False):
        print(f"committing and tagging this version...")
        repo = Repo(self.path_to_git_repo)
        tags = sorted(repo.tags, key=lambda t: t.commit.committed_datetime)
        latest_tag = tags[-1]
        if latest_tag.name == f'v{self.version}':
            raise ValueError(f'Current version ({latest_tag.name}) already uploaded. Set a new version.')

        try:

            repo.git.add('.')
            count_staged_files = len(repo.index.diff("HEAD"))

            if count_staged_files != 0:
                commit = repo.index.commit(self.commit_message)
                new_tag = repo.create_tag(f'{self.commit_tag}', ref=commit)

                if push:
                    print("pushing...")
                    repo.git.push('origin', repo.active_branch, new_tag)
                    print('done')
            else:
                print('nothing to commit.')

        except Exception as e:
            print(e)


    def docs_update(self):
        print('building documentation and uploading to gh-pages...')
        docs_dir = self.path_to_git_repo + '/docs'

        ignored_files = []
        ignored_files.append(self.package + '/playground/')
        ignored_files.append(self.package + '/examples/')
        ignored_files.append(self.package + '/tests/')

        build_docs_with_sphinx = ["sphinx-apidoc", "-f", "-o", "docs/source", self.package]
        build_docs_with_sphinx.extend(ignored_files)
        p = subprocess.check_output(build_docs_with_sphinx)
        p = subprocess.check_output(["make", "buildandcommithtml", "-j8"], cwd=docs_dir)


if __name__ == '__main__':



    parser = argparse.ArgumentParser(description='prepare tag for travis-ci')
    parser.add_argument('--push', '-p', action='store_true', help='push the tag to github')
    parser.add_argument("--docs", '-d', action='store_true', help='bump also documentation')

    args = parser.parse_args()

    package_name = "horizon"
    updr = Updater(package_name)
    updr.git_update(args.push)
    
    if args.docs:
        updr.docs_update()






