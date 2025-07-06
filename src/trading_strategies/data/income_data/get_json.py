import json
import pprint

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


def load_file(path):
    with open(path, 'r') as f:
        content = json.load(f)
        return json.dumps(content)
# Press the green button in the gutter to run the script.


if __name__ == '__main__':
    data = json.loads(load_file("C:/Users/Terry/PycharmProjects/new_test/data/income_data/From AOS to AEE"))
    # print(type(data))

    # 全部信息输出
    # pprint.pprint(data)

    # 只针对年报 全部公司
    for keys, values in data.items():
        # print()
        for i in values['annualReports']:
            pprint.pprint(i['totalRevenue'])
        # pprint.pprint(values['annualReports'][0]['totalRevenue'])