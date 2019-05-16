from testlib.androidframework.adb_utils import AdbUtils
from string import Template
import re

class LogcatMessagingGateway(object):
    logcat_filter_cmd_template = Template("logcat -d | grep '$marker'")

    @staticmethod
    def filter_relevant_logcat_lines(relevant_marker):
        filter_cmd = LogcatMessagingGateway\
            .logcat_filter_cmd_template.substitute(marker=relevant_marker)
        relevant_lines = AdbUtils._run_adb_cmd(filter_cmd, add_ticks=False)
        return relevant_lines


class ApiTestsMessagingGateway(LogcatMessagingGateway):
    api_tests_logcat_marker = "API Tests"
    message_start_marker = "<api_tests_message_start"
    message_end_marker = "<api_tests_message_end"
    message_line_marker = "<message_line"
    message_id_extract_regex = "id=\'(.+)\'"
    message_content_extract_regex = "<message_line id=\'.+\'>(.+)"

    def __init__(self):
        self.complete_messages = []
        self.incomplete_messages = []
        self.get_logcat_messages()

    @staticmethod
    def is_api_tests_message_line(msg_line):
        return ApiTestsMessagingGateway.message_end_marker in msg_line or\
            ApiTestsMessagingGateway.message_start_marker in msg_line or\
            ApiTestsMessagingGateway.message_line_marker in msg_line

    @staticmethod
    def get_id_from_message_line(msg_line):
        ids = re.findall(ApiTestsMessagingGateway.message_id_extract_regex, msg_line)
        if len(ids) == 0:
            return None
        return ids[0]

    @staticmethod
    def get_message_content_from_line(msg_line):
        content = re.findall(ApiTestsMessagingGateway.message_content_extract_regex, msg_line)
        if len(content) == 0:
            return None
        return content[0]

    @staticmethod
    def is_message_start_line(msg_line):
        return ApiTestsMessagingGateway.message_start_marker in msg_line

    @staticmethod
    def is_message_end_line(msg_line):
        return ApiTestsMessagingGateway.message_end_marker in msg_line

    def handle_message_start_line(self, msg_line, msg_id):
        message = Message(msg_id)
        self.incomplete_messages.append(message)

    def handle_message_end_line(self, msg_line, msg_id):
        # find the first currently open message with the same id and close it
        # if there are more messages with the same id, things may not work as desired
        # we cannot match an ending with the proper beginning if ids are not unique
        for msg in self.incomplete_messages:
            if msg.id == msg_id:
                self.complete_messages.append(msg)
                self.incomplete_messages.remove(msg)
                return

    def handle_message_content_line(self, msg_line, msg_id):
        for msg in self.incomplete_messages:
            if msg.id == msg_id:
                line_content = ApiTestsMessagingGateway.get_message_content_from_line(msg_line)
                msg.add_line(line_content)

    def handle_message_line(self, msg_line):
        line_msg_id = ApiTestsMessagingGateway.get_id_from_message_line(msg_line)
        if line_msg_id is None:
            return
        if ApiTestsMessagingGateway.is_message_start_line(msg_line):
            self.handle_message_start_line(msg_line, line_msg_id)
        elif ApiTestsMessagingGateway.is_message_end_line(msg_line):
            self.handle_message_end_line(msg_line, line_msg_id)
        else:  # is message content line
            self.handle_message_content_line(msg_line, line_msg_id)

    def get_logcat_messages(self):
        relevant_lines = ApiTestsMessagingGateway\
            .filter_relevant_logcat_lines(ApiTestsMessagingGateway.api_tests_logcat_marker)
        for msg_line in relevant_lines.splitlines():
            if ApiTestsMessagingGateway.is_api_tests_message_line(msg_line):
                self.handle_message_line(msg_line)

    def get_message_by_id(self, msg_id):
        result = []
        for msg in self.complete_messages:
            if msg.id == msg_id:
                result.append(msg)
        return result

    def get_latest_message(self):
        return self.complete_messages[-1]

    def get_latest_message_with_id(self, msg_id):
        return self.get_message_by_id(msg_id)[-1]


class Message(object):
    def __init__(self, msg_id):
        self.id = msg_id
        self.content_lines = []

    def add_line(self, msg_line):
        self.content_lines.append(msg_line)

    def get_content_as_string(self):
        return "\n".join(self.content_lines)

    def get_content_as_on_line_string(self):
        return "".join(self.content_lines)

if __name__ == "__main__":
    # Example Usage
    api_tests_msg = ApiTestsMessagingGateway()
    print api_tests_msg.get_latest_message().get_content_as_string()
    print api_tests_msg.get_latest_message_with_id("AppDirsMsgID").get_content_as_string()
    for line in api_tests_msg.get_latest_message_with_id("AppDirsMsgID").content_lines:
        print line
