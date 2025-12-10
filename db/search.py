import sqlite3

conn = sqlite3.connect("data.db")
cur = conn.cursor()

cur.execute("""
    SELECT *
    FROM event_data
    ORDER BY timestamp DESC
    LIMIT 1
""")
row = cur.fetchone()

print(row)