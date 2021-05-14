import re #gex as re
from PyQt5.QtWidgets import QApplication, QSpinBox, QDoubleSpinBox, QLineEdit, QPushButton, QVBoxLayout
from PyQt5.QtGui import QKeyEvent, QValidator, QIntValidator, QDoubleValidator, QRegExpValidator, QRegularExpressionValidator
from PyQt5.QtCore import Qt, QRegExp

def partial_match(regex, string, flags=0, op=re.match):
    """
    Matches a regular expression to a string incrementally, retaining the
    best substring matched.
    :param regex:   The regular expression to apply to the string.
    :param string:  The target string.
    :param flags:   re module flags, e.g.: `re.I`
    :param op:      Either of re.match (default) or re.search.
    :return:        The substring of the best partial match.
    """
    # if regex[-1] == '|':
    #     regex = regex[:-1]

    # split all formula
    # re.sub("[\(].*?[\)]", "", x)

    m = op(regex, string)
    if m:
        return m.group(0)
    final = None

    for i in range(1, len(regex) + 1):
        try:
            if regex[:i][-1] != '|' and regex[:i][-1] != '\\':
                m = op(regex[:i], string, flags)
                # print('iter {}: try {} using {}. SOLUTION: {}'.format(i, regex[:i], string, m))
            if m:
                final = m.group(0)
        except re.error:
            print('ERROR at iter {}: {}'.format(i, regex[:i]))
            pass

    return final

class MySecondValidator(QValidator):
    def __init__(self, parent):
        QValidator.__init__(self, parent)
        self.parent = parent

        self.perfect_matches = set()

        self.var = dict()
        self.variables = ['x', 'y', 'culo']

        binary_op = ['\+', '\-', '\*', '\/', '\^']
        # operators regex
        op_re = '|'.join(binary_op)

        var = ['x', 'y', 'culo']
        self.var_re = '|'.join(var)
        var_re_group = '(' + '?P<var_name>' + self.var_re + ')'
        self.slice_re = '\[\d+:\d+\]'

        slice_re_group = '(' + '?P<var_slice>' + self.slice_re + ')'


        # all the variables (var_regex + var_slice_regex)
        self.all_var_regex = '(' + var_re_group + ')' + slice_re_group + '?' + '(' + op_re + ')'
        self.all_var_regex_mod = '(' + self.var_re + ')' + '(' + self.slice_re + ')?' + '(' + op_re + ')'

        self.repeating_regex = '(' + self.all_var_regex + ')+'
        self.repeating_regex_mod = '(' + self.all_var_regex_mod + ')+'

        print('full_rule', self.repeating_regex)
        print('rule:', self.repeating_regex_mod)

    def checkPerfectMatch(self, text):
        for match in self.perfect_matches:
            if match not in text:
                print('match {} no longer in text {}. Removing!'.format(match, text))
                self.perfect_matches.remove(match)
                break

    def validate(self, text, pos):

        penis = re.compile(self.repeating_regex)  # r'(-?\d+)x\^\d+ ([+-]\d+)x ([+-]\d+)'
        matches = penis.match(text)
        test_text = text

        for match in self.perfect_matches:
            test_text = test_text.replace(match, '', 1)

        print('TESTING WITH: {}'.format(test_text))
        if test_text == '':
            print('entered intermediate (empty string)')
            return QValidator.Intermediate, text, pos

        print('CURRENT MATCH:', matches)
        if not matches or matches.group(0) != text:
            intermediate_text = test_text
            print('-------- entered intermediate: ---------')
            if not matches:
                print('no matches found, but there is still hope')
            else:
                print('{} != {}'.format(matches.group(0), text))

            result = re.split('[()]', self.repeating_regex_mod)
            print('splitted result:', result)
            # todo something here!
            if result:
                for regex_portion in result:
                    if regex_portion == '':
                        # print('starting with ()')
                        continue
                    if regex_portion == '?':
                        # print('ignoring "?"')
                        continue
                    if regex_portion == '+':
                        # print('ignoring "+')
                        continue

                    print('regex current portion:', regex_portion)


                    print('trying {} with {}'.format(regex_portion, intermediate_text))
                    partial_final = partial_match(regex_portion, intermediate_text)
                    print('Found a partial match:', partial_final)

                    if partial_final:
                        if partial_final != intermediate_text:
                            print('Text {} exceed partial match {}'.format(intermediate_text, partial_final))
                            print('Checking if last partial match {} completely matches last slice {}'.format(partial_final, regex_portion))
                            old_intermediate_text = intermediate_text
                            if re.match(regex_portion, partial_final):
                                intermediate_text = intermediate_text.replace(partial_final, '', 1)
                                print('Success! Continuing to next slice, using {} instead of {}'.format(intermediate_text, old_intermediate_text))
                                continue
                            else:
                                print('Fail!')
                                break
                        else:
                            print('--------- exited with signal "INTERMEDIATE" ({} == {}) ---------'.format(partial_final, intermediate_text))
                            self.checkPerfectMatch(self, text)
                            return QValidator.Intermediate, text, pos
                    else:
                        print('--------- exited with signal "INVALID" (partial final: {}) ---------'.format(partial_final))
                        return QValidator.Invalid, text, pos
                print('--------- exited with signal "INVALID" (No more slices.) ---------')
                return QValidator.Invalid, text, pos
        else:
            print('exited with signal "ACCEPTABLE"')
            print(matches.groupdict())
            self.perfect_matches.add(test_text)
            print('ADDING {} to ignore list: {}'.format(test_text, self.perfect_matches))
            print('==============================================================================================')
            self.checkPerfectMatch(self, text)
            return QValidator.Acceptable, text, pos

def remove_groups(s, group_dict):
    print(s)
    variables = ['x','y', 'culo']

    groups_dict = dict(var_name='|'.join(variables), var_slice='\[\d+:\d+\]', porcodio='dio|cane', salamino='concotto|talamide')
    # find group name
    group_names = re.findall('(?<=\(\?P<)\w+', s)
    if group_names:
        found_groups = dict()
        for name in group_names:
            if name in groups_dict:
                found_groups[name] = groups_dict[name]
            else:
                print('no val for group:', name)
                return False
    else:
        return False

    return found_groups


class ValidatorLine(QLineEdit):
    def __init__(self):
        QLineEdit.__init__(self)


        self.setMinimumWidth(300)
        self.validator = MySecondValidator(self)
        self.setValidator(self.validator)


if __name__ == '__main__':
    import sys

    # regex = 'a(culodio)()(sakamoto)pinzeruolo solo'



    app = QApplication(sys.argv)

    infbox = ValidatorLine()
    # hslider.setEmitWhileMoving(True)
    infbox.show()
    sys.exit(app.exec_())

